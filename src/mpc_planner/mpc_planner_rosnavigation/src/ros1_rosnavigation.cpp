#include <mpc_planner_rosnavigation/ros1_rosnavigation.h>

#include <mpc_planner_util/math.h>
#include <mpc_planner_util/parameters.h>
#include <mpc_planner/planner.h>

#include <pluginlib/class_list_macros.h>

#include <mpc_planner_util/visuals.h>
#include <mpc_planner_util/convertions.h>

#include <std_msgs/Empty.h>

using namespace MPCPlanner;

PLUGINLIB_EXPORT_CLASS(local_planner::ROSNavigationPlanner, nav_core::BaseLocalPlanner)

namespace local_planner
{

    ROSNavigationPlanner::ROSNavigationPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

    ROSNavigationPlanner::ROSNavigationPlanner(std::string name, tf2_ros::Buffer *tf,
                                               costmap_2d::Costmap2DROS *costmap_ros)
        : costmap_ros_(NULL), tf_(NULL), initialized_(false)
    {
        initialize(name, tf, costmap_ros);
    }

    void ROSNavigationPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                                          costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            ros::NodeHandle nh("~/" + name);

            tf_ = tf;

            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            _data.costmap = costmap_;

            initialized_ = true;

            ROS_INFO_STREAM("Started ROSNavigation Planner!!!");

            VISUALS.init(&general_nh_);

            // Initialize the configuration
            Configuration::getInstance().initialize(SYSTEM_CONFIG_PATH(__FILE__, "settings"));

            _data.robot_area = {Disc(0., CONFIG["robot_radius"].as<double>())};

            // Initialize the planner
            _planner = std::make_unique<Planner>();

            // Initialize the ROS interface
            initializeSubscribersAndPublishers(nh);

            startEnvironment();

            ROS_INFO_STREAM("========================================");
        }
    }

    ROSNavigationPlanner::~ROSNavigationPlanner()
    {
        ROS_INFO_STREAM("Stopped ROSNavigation Planner");
    }

    bool ROSNavigationPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        // check if plugin is initialized
        if (!initialized_)
        {
            ROS_ERROR("planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        global_plan_.clear();
        global_plan_ = orig_global_plan;

        return true;
    }

    bool ROSNavigationPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        auto path = boost::make_shared<nav_msgs::Path>();
        path->poses = global_plan_;
        pathCallback(path);

        if (_rotate_to_goal)
            rotateToGoal(cmd_vel);
        else
            loop(cmd_vel);

        return true;
    }

    void ROSNavigationPlanner::initializeSubscribersAndPublishers(ros::NodeHandle &nh)
    {
        ROS_INFO_STREAM("initializeSubscribersAndPublishers");

        _state_sub = nh.subscribe<nav_msgs::Odometry>(
            "/input/state", 5,
            boost::bind(&ROSNavigationPlanner::stateCallback, this, _1));

        _goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(
            "/input/goal", 1,
            boost::bind(&ROSNavigationPlanner::goalCallback, this, _1));

        _cmd_pub = nh.advertise<geometry_msgs::Twist>(
            "/output/command", 1);

        // Environment Reset
        _reset_goal_pub = nh.advertise<std_msgs::Empty>("/lmpcc/reset_environment", 1);
        _reset_simulation_client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
        _reset_ekf_client = nh.serviceClient<robot_localization::SetPose>("/set_pose");
    }

    void ROSNavigationPlanner::startEnvironment() // TODO MOAI
    {
        ROS_INFO_STREAM("Starting pedestrian simulator");
        for (int i = 0; i < 1; i++)
        {
            ROS_INFO_STREAM_THROTTLE(3.0, "Waiting for goal to start");
            ros::Duration(1.0).sleep();

            _reset_goal_pub.publish(std_msgs::Empty());
        }
        _enable_output = CONFIG["enable_output"].as<bool>();
        ROS_INFO_STREAM("Environment ready.");
    }

    bool ROSNavigationPlanner::isGoalReached()
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        bool goal_reached = _planner->isObjectiveReached(_state, _data) && !done_; // Activate once
        if (goal_reached)
        {
            ROS_INFO_STREAM("Goal Reached!");
            done_ = true;
            reset();
        }

        return goal_reached;
    }

    void ROSNavigationPlanner::rotateToGoal(geometry_msgs::Twist &cmd_vel)
    {
        ROS_INFO_STREAM_THROTTLE(1500, "Rotating to the goal");
        if (!_data.goal_received)
        {
            ROS_INFO_STREAM("Waiting for the goal");
            return;
        }
        double goal_angle = 0.;

        if (_data.reference_path.x.size() > 2)
            goal_angle = std::atan2(_data.reference_path.y[2] - _state.get("y"), _data.reference_path.x[2] - _state.get("x"));
        else
            goal_angle = std::atan2(_data.goal(1) - _state.get("y"), _data.goal(0) - _state.get("x"));

        double angle_diff = goal_angle - _state.get("psi");

        if (angle_diff > M_PI)
            angle_diff -= 2 * M_PI;

        geometry_msgs::Twist cmd;
        if (std::abs(angle_diff) > M_PI / 4.)
        {
            cmd_vel.linear.x = 0.0;
            if (_enable_output)
                cmd_vel.angular.z = 1.5 * RosTools::sgn(angle_diff);
            else
                cmd_vel.angular.z = 0.;
        }
        else
        {
            ROS_INFO_STREAM("Robot rotated and is ready to follow the path");
            _rotate_to_goal = false;
        }
    }

    void ROSNavigationPlanner::loop(geometry_msgs::Twist &cmd_vel)
    {

        // Copy data for thread safety
        RealTimeData data = _data;
        State state = _state;

        data.planning_start_time = std::chrono::system_clock::now();

        if (CONFIG["debug_output"].as<bool>())
            ROS_INFO_STREAM("============= Loop =============");

        auto output = _planner->solveMPC(state, data);

        if (CONFIG["debug_output"].as<bool>())
            ROS_INFO_STREAM("Success: " << output.success);

        geometry_msgs::Twist cmd;
        if (_enable_output && output.success)
        {
            // Publish the command
            cmd_vel.linear.x = _planner->getSolution(1, "v");  // = x1
            cmd_vel.angular.z = _planner->getSolution(0, "w"); // = u0
        }
        else
        {
            double deceleration = CONFIG["deceleration_at_infeasible"].as<double>();
            double velocity_after_braking;
            double velocity;
            double dt = 1. / CONFIG["control_frequency"].as<double>();

            velocity = _state.get("v");
            velocity_after_braking = velocity - deceleration * dt;   // Brake with the given deceleration
            cmd_vel.linear.x = std::max(velocity_after_braking, 0.); // Don't drive backwards when braking
            cmd_vel.angular.z = 0.0;
        }
        _cmd_pub.publish(cmd);

        if (output.success)
        {
            _planner->visualize(state, data);
        }
        if (CONFIG["debug_output"].as<bool>())
            ROS_INFO_STREAM("============= End Loop =============");
    }

    void ROSNavigationPlanner::stateCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (CONFIG["debug_output"].as<bool>())
            ROS_INFO_STREAM("State callback");
        _state.set("x", msg->pose.pose.position.x);
        _state.set("y", msg->pose.pose.position.y);
        _state.set("psi", RosTools::quaternionToAngle(msg->pose.pose.orientation));
        _state.set("v", std::sqrt(std::pow(msg->twist.twist.linear.x, 2.) + std::pow(msg->twist.twist.linear.y, 2.)));

        if (std::abs(msg->pose.pose.orientation.x) > (M_PI / 8.) || std::abs(msg->pose.pose.orientation.y) > (M_PI / 8.))
        {
            ROS_WARN_STREAM("Detected flipped robot. Resetting.");
            reset(false); // Reset without success
        }
    }

    void ROSNavigationPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (CONFIG["debug_output"].as<bool>())
            ROS_INFO_STREAM("Goal callback");

        _data.goal(0) = msg->pose.position.x;
        _data.goal(1) = msg->pose.position.y;
        _data.goal_received = true;

        _rotate_to_goal = true;
    }

    bool ROSNavigationPlanner::isPathTheSame(const nav_msgs::Path::ConstPtr &msg)
    {
        // Check if the path is the same
        if (_data.reference_path.x.size() != msg->poses.size())
            return false;

        // Check up to the first two points
        int num_points = std::min(2, (int)_data.reference_path.x.size());
        for (int i = 0; i < num_points; i++)
        {
            if (!_data.reference_path.pointInPath(i, msg->poses[i].pose.position.x, msg->poses[i].pose.position.y))
                return false;
        }
        return true;
    }

    void ROSNavigationPlanner::pathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        if (CONFIG["debug_output"].as<bool>())
            ROS_INFO_STREAM("Path callback");

        int downsample = CONFIG["downsample_path"].as<double>();

        if (isPathTheSame(msg) || msg->poses.size() < downsample + 1)
            return;

        _data.reference_path.clear();

        int count = 0;
        for (auto &pose : msg->poses)
        {
            if (count % downsample == 0 || count == msg->poses.size() - 1) // Todo
            {
                _data.reference_path.x.push_back(pose.pose.position.x);
                _data.reference_path.y.push_back(pose.pose.position.y);
                _data.reference_path.psi.push_back(RosTools::quaternionToAngle(pose.pose.orientation));
            }
            count++;
        }

        _planner->onDataReceived(_data, "reference_path");
    }

    void ROSNavigationPlanner::reset(bool success)
    {
        ROS_INFO_STREAM("Resetting");
        // boost::mutex::scoped_lock l(_reset_mutex);

        _reset_simulation_client.call(_reset_msg);
        _reset_ekf_client.call(_reset_pose_msg);
        _reset_goal_pub.publish(std_msgs::Empty());

        _planner->reset(_state, _data, success);
        _data.costmap = costmap_;

        ros::Duration(1.0 / CONFIG["control_frequency"].as<double>()).sleep();

        done_ = false;
        _rotate_to_goal = false;
    }
} // namespace local_planner