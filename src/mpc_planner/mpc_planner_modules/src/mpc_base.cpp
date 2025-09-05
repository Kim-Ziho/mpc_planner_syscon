#include <mpc_planner_modules/mpc_base.h>
#include <mpc_planner_util/parameters.h>

#define WEIGHT_PARAMS {"acceleration", "angular_velocity", "velocity", "reference_velocity"}

namespace MPCPlanner
{

  MPCBaseModule::MPCBaseModule(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::OBJECTIVE, solver, "mpc_base")
  {
    _weight_names = WEIGHT_PARAMS;
  }

  void MPCBaseModule::update(State &state, const RealTimeData &data, ModuleData &module_data)
  {
    (void)state;
    (void)data;
    (void)module_data;
  }

  void MPCBaseModule::setParameters(const RealTimeData &data, const ModuleData &module_data, int k)
  {
    (void)data;
    (void)module_data;

    if (k == 0)
      if (CONFIG["debug_output"].as<bool>())
        ROS_INFO_STREAM("MPCBaseModule::setParameters()");

    for (auto &weight : _weight_names)
    {
      _solver->setParameter(k, weight, CONFIG["weights"][weight].as<double>());
    }
  }
} // namespace MPCPlanner