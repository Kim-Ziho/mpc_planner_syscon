#include <mpc_planner/planner.h>

#include <mpc_planner_modules/mpc_base.h>
#include <mpc_planner_modules/contouring.h>
#include <mpc_planner_modules/decomp_constraints.h>

#include <mpc_planner_types/realtime_data.h>
#include <mpc_planner_solver/acados_solver_interface.h>

#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/data_visualization.h>

#include <ros_tools/visuals.h>

namespace MPCPlanner
{

    Planner::Planner()
    {
        // Initialize the solver
        _solver = std::make_shared<Solver>();
        _solver->reset();

        _modules.emplace_back(std::make_shared<MPCBaseModule>(_solver));
        _modules.emplace_back(std::make_shared<Contouring>(_solver));
        _modules.emplace_back(std::make_shared<DecompConstraints>(_solver));
    }

    // Given real-time data, solve the MPC problem
    PlannerOutput Planner::solveMPC(State &state, RealTimeData &data)
    {
        if (CONFIG["debug_output"].as<bool>())
            ROS_INFO_STREAM("Planner::solveMPC");
        bool was_feasible = _output.success;
        _output = PlannerOutput(_solver->dt, _solver->N);

        _module_data = ModuleData(); // Reset module data

        // Check if all modules have enough data
        _is_data_ready = true;
        std::string missing_data;
        for (auto &module : _modules)
            _is_data_ready = _is_data_ready & module->isDataReady(data, missing_data);

        if (!_is_data_ready)
        {
            ROS_WARN_STREAM_THROTTLE(3000, "Data is not ready, missing " + missing_data + "\b");

            _output.success = false;
            return _output;
        }
        else if (_was_reset)
        {
            _was_reset = false;
        }

        if (CONFIG["debug_output"].as<bool>())
            ROS_INFO_STREAM("Data checked");

        int exit_flag;
        {
            // Set the initial guess
            bool shift_forward = CONFIG["shift_previous_solution_forward"].as<bool>() &&
                                 CONFIG["enable_output"].as<bool>();
            if (was_feasible)
                _solver->initializeWarmstart(state, shift_forward);
            else
            {
                // _solver->initializeWithState(state);
                _solver->initializeWithBraking(state);
            }

            _solver->setXinit(state); // Set the initial state

            // Update all modules
            {
                if (CONFIG["debug_output"].as<bool>())
                    ROS_INFO_STREAM("Updating modules");

                for (auto &module : _modules)
                    module->update(state, data, _module_data);
            }

            {
                if (CONFIG["debug_output"].as<bool>())
                    ROS_INFO_STREAM("Setting parameters");

                for (int k = 0; k < _solver->N; k++)
                {
                    for (auto &module : _modules)
                    {
                        module->setParameters(data, _module_data, k);
                    }
                }
            }

            _solver->loadWarmstart();

            // set solver_timeout
            std::chrono::duration<double> used_time = std::chrono::system_clock::now() - data.planning_start_time;
            _solver->_params.solver_timeout = 1. / CONFIG["control_frequency"].as<double>() - used_time.count() - 0.006;

            // Solve MPC
            if (CONFIG["debug_output"].as<bool>())
                ROS_INFO_STREAM("Solve optimization");
            {
                exit_flag = _solver->solve();
            }
        }

        if (exit_flag != 1)
        {
            _output.success = false;
            ROS_WARN_STREAM_THROTTLE(500, "MPC failed: " + _solver->explainExitFlag(exit_flag));

            return _output;
        }

        _output.success = true;
        for (int k = 1; k < _solver->N; k++)
            _output.trajectory.add(_solver->getOutput(k, "x"), _solver->getOutput(k, "y"));

        // if (_output.success && CONFIG["debug_limits"].as<bool>())
        // _solver->printIfBoundLimited();

        if (CONFIG["debug_output"].as<bool>())
            ROS_INFO_STREAM("Planner::solveMPC done");

        return _output;
    }

    double Planner::getSolution(int k, std::string &&var_name) const
    {
        return _solver->getOutput(k, std::forward<std::string>(var_name));
    }

    void Planner::onDataReceived(RealTimeData &data, std::string &&data_name)
    {
        for (auto &module : _modules)
            module->onDataReceived(data, std::forward<std::string>(data_name));
    }

    void Planner::visualize(const State &state, const RealTimeData &data)
    {
        if (CONFIG["debug_output"].as<bool>())
            ROS_INFO_STREAM("Planner::visualize");
        (void)state;

        for (auto &module : _modules)
            module->visualize(data, _module_data);

        visualizeTrajectory(_output.trajectory, "planned_trajectory", true, 0.2);
        visualizeRobotArea(state.getPos(), state.get("psi"), data.robot_area, "robot_area", true);

        if (CONFIG["debug_output"].as<bool>())
            ROS_INFO_STREAM("Planner::visualize Done");
    }

    void Planner::reset(State &state, RealTimeData &data, bool success)
    {
        _solver->reset(); // Reset the solver

        for (auto &module : _modules) // Reset modules
            module->reset();

        state = State(); // Reset the state
        data.reset();    // Reset the data
        _was_reset = true;
    }

    bool Planner::isObjectiveReached(const State &state, const RealTimeData &data) const
    {
        bool objective_reached = true;
        for (auto &module : _modules)
            objective_reached = objective_reached && module->isObjectiveReached(state, data);
        return objective_reached;
    }
}