#ifndef __ROS1_ROSNAVIGATION_PLANNER_H__
#define __ROS1_ROSNAVIGATION_PLANNER_H__

#include <nav_core/base_local_planner.h>

#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>

#include <mpc_planner_types/realtime_data.h>
#include <mpc_planner_solver/state.h>

#include <ros/ros.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <std_srvs/Empty.h>
#include <robot_localization/SetPose.h>

using namespace MPCPlanner;

namespace MPCPlanner
{
    class Planner;
}
namespace local_planner
{
    class ROSNavigationPlanner : public nav_core::BaseLocalPlanner
    {

    public:
        // ROSNavigationPlanner(ros::NodeHandle &nh);

        // NAVIGATION STACK
        ROSNavigationPlanner();
        ROSNavigationPlanner(std::string name, tf2_ros::Buffer *tf,
                             costmap_2d::Costmap2DROS *costmap_ros);

        ~ROSNavigationPlanner();

        void initialize(std::string name, tf2_ros::Buffer *tf,
                        costmap_2d::Costmap2DROS *costmap_ros);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

        bool isGoalReached();

        // OTHER STANDARD FUNCTIONS
        void initializeSubscribersAndPublishers(ros::NodeHandle &nh);

        void startEnvironment();

        void rotateToGoal(geometry_msgs::Twist &cmd_vel);
        void loop(geometry_msgs::Twist &cmd_vel);

        void stateCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void pathCallback(const nav_msgs::Path::ConstPtr &msg);

        void reset(bool success = true);

    private:
        // ROS Navigation stack
        costmap_2d::Costmap2DROS *costmap_ros_;
        costmap_2d::Costmap2D *costmap_;

        tf2_ros::Buffer *tf_;
        bool initialized_;
        std::vector<geometry_msgs::PoseStamped> global_plan_; //!< Store the current global plan

        ros::NodeHandle general_nh_;

        // Other
        bool done_{false};
        bool _rotate_to_goal{false};

        std::unique_ptr<Planner> _planner;

        RealTimeData _data;
        State _state;

        bool _enable_output{true};

        // Subscribers and publishers
        ros::Subscriber _state_sub, _state_pose_sub;
        ros::Subscriber _goal_sub;

        ros::Publisher _cmd_pub;

        ros::Time _prev_stamp;

        std_srvs::Empty _reset_msg;
        robot_localization::SetPose _reset_pose_msg;
        ros::Publisher _reset_goal_pub;
        ros::ServiceClient _reset_simulation_client;
        ros::ServiceClient _reset_ekf_client;

        bool isPathTheSame(const nav_msgs::Path::ConstPtr &path);
    };
} // namespace local_planner
#endif // __ROS1_ROSNAVIGATION_PLANNER_H__
