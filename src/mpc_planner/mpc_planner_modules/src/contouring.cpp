#include "mpc_planner_modules/contouring.h"

#include <mpc_planner_solver/mpc_planner_parameters.h>

#include <mpc_planner_util/parameters.h>
#include <mpc_planner_util/data_visualization.h>

#include <mpc_planner_util/visuals.h>
#include <mpc_planner_util/math.h>

#include <algorithm>

namespace MPCPlanner
{
  Contouring::Contouring(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::OBJECTIVE, solver, "contouring")
  {
    ROS_INFO_STREAM("Contouring::initialize");
    _n_segments = CONFIG["contouring"]["num_segments"].as<int>();
    _add_road_constraints = CONFIG["contouring"]["add_road_constraints"].as<bool>();

    ROS_INFO_STREAM("Contouring::initialize done");
  }

  void Contouring::update(State &state, const RealTimeData &data, ModuleData &module_data)
  {
    (void)data;

    if (CONFIG["debug_output"].as<bool>())
      ROS_INFO_STREAM("contouring::update()");

    // Update the closest point
    double closest_s;
    _spline->findClosestPoint(state.getPos(), _closest_segment, closest_s);

    if (module_data.path.get() == nullptr && _spline.get() != nullptr)
      module_data.path = _spline;

    state.set("spline", closest_s); // We need to initialize the spline state here

    if (CONFIG["debug_output"].as<bool>())
      state.print();

    module_data.current_path_segment = _closest_segment;
  }

  void Contouring::setParameters(const RealTimeData &data, const ModuleData &module_data, int k)
  {
    (void)data;
    (void)module_data;

    // Retrieve weights once
    static double contouring_weight, lag_weight;
    static double terminal_angle_weight, terminal_contouring_weight;
    if (k == 0)
    {
      if (CONFIG["debug_output"].as<bool>())
        ROS_INFO_STREAM("contouring::setParameters()");
      contouring_weight = CONFIG["weights"]["contour"].as<double>();
      lag_weight = CONFIG["weights"]["lag"].as<double>();

      terminal_angle_weight = CONFIG["weights"]["terminal_angle"].as<double>();
      terminal_contouring_weight = CONFIG["weights"]["terminal_contouring"].as<double>();
    }

    {
      setSolverParameterContour(k, _solver->_params, contouring_weight);
      setSolverParameterLag(k, _solver->_params, lag_weight);

      setSolverParameterTerminalAngle(k, _solver->_params, terminal_angle_weight);
      setSolverParameterTerminalContouring(k, _solver->_params, terminal_contouring_weight);
    }

    setSplineParameters(k);
  }

  void Contouring::setSplineParameters(int k)
  {
    double ax, bx, cx, dx;
    double ay, by, cy, dy;
    double start;

    for (int i = 0; i < _n_segments; i++)
    {
      int index = _closest_segment + i;

      _spline->getParameters(index,
                             ax, bx, cx, dx,
                             ay, by, cy, dy);

      start = _spline->getSegmentStart(index);

      /** @note: We use the fast loading interface here as we need to load many parameters */
      setSolverParameterSplineXA(k, _solver->_params, ax, i);
      setSolverParameterSplineXB(k, _solver->_params, bx, i);
      setSolverParameterSplineXC(k, _solver->_params, cx, i);
      setSolverParameterSplineXD(k, _solver->_params, dx, i);

      setSolverParameterSplineYA(k, _solver->_params, ay, i);
      setSolverParameterSplineYB(k, _solver->_params, by, i);
      setSolverParameterSplineYC(k, _solver->_params, cy, i);
      setSolverParameterSplineYD(k, _solver->_params, dy, i);

      // Distance where this spline starts
      setSolverParameterSplineStart(k, _solver->_params, start, i);
    }
  }

  void Contouring::onDataReceived(RealTimeData &data, std::string &&data_name)
  {
    if (data_name == "reference_path")
    {
      if (CONFIG["debug_output"].as<bool>())
        ROS_INFO_STREAM("Received Reference Path");

      // Construct a spline from the given points
      if (data.reference_path.s.empty())
        _spline = std::make_shared<RosTools::Spline2D>(data.reference_path.x, data.reference_path.y);
      else
        _spline = std::make_shared<RosTools::Spline2D>(data.reference_path.x, data.reference_path.y, data.reference_path.s);

      _closest_segment = -1;
    }
  }

  bool Contouring::isDataReady(const RealTimeData &data, std::string &missing_data)
  {
    if (data.reference_path.x.empty())
      missing_data += "Reference Path ";

    return !data.reference_path.x.empty();
  }

  bool Contouring::isObjectiveReached(const State &state, const RealTimeData &data)
  {
    (void)data;

    if (!_spline)
      return false;

    // Check if we reached the end of the spline
    return RosTools::distance(state.getPos(), _spline->getPoint(_spline->parameterLength())) < 1.0;
  }

  void Contouring::visualize(const RealTimeData &data, const ModuleData &module_data)
  {
    if (_spline.get() == nullptr)
      return;

    if (data.reference_path.empty())
      return;

    if (CONFIG["debug_output"].as<bool>())
      ROS_INFO_STREAM("Contouring::Visualize");

    visualizeReferencePath(data, module_data);
  }

  // Move to data visualization
  void Contouring::visualizeReferencePath(const RealTimeData &data, const ModuleData &module_data)
  {
    (void)module_data;

    visualizePathPoints(data.reference_path, _name + "/path", false);
    visualizeSpline(*_spline, _name + "/path", true);
  }

  void Contouring::reset()
  {
    _spline.reset();
    _closest_segment = 0;
  }
} // namespace MPCPlanner