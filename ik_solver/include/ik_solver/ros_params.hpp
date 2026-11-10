#ifndef IK_SOLVER__ROS_PARAMS_HPP
#define IK_SOLVER__ROS_PARAMS_HPP

/**
 * @file ros_params.hpp
 * @brief ROS 2 glue for ik_solver_core: logger adapter and parameters.
 *
 * ik_solver_core is ROS-free: it logs through an ik_solver::LoggerPtr and reads its
 * parameters from an ik_solver::ParamMap. This header provides
 * - RclcppLogger, an ik_solver::Logger writing to an rclcpp::Logger;
 * - paramsFromNode(), which builds a ParamMap from the ROS 2 parameters of a node
 *   under a prefix.
 *
 * Example (solver used inside a node, parameters under "ur_ik_solver."):
 * @code
 * solver->setLogger(std::make_shared<ik_solver::RclcppLogger>(node.get_logger().get_child("ik_solver")));
 * ik_solver::ParamMap params = ik_solver::paramsFromNode(node, "ur_ik_solver");
 * params["robot_description"] = urdf;
 * solver->config(params);
 * @endcode
 */

#include <ik_solver_core/logger.h>
#include <ik_solver_core/params.h>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace ik_solver
{
/**
 * @brief ik_solver::Logger writing to an rclcpp::Logger. TRACE is mapped to
 * DEBUG; the message keeps the function and line of the solver code.
 */
class RclcppLogger : public ik_solver::Logger
{
public:
  explicit RclcppLogger(const rclcpp::Logger& logger) : logger_(logger)
  {
  }

  bool enabled(ik_solver::LogLevel level) const override
  {
    return rcutils_logging_logger_is_enabled_for(logger_.get_name(), severity(level));
  }

  void log(ik_solver::LogLevel level, const char* func, int line, const std::string& msg) override
  {
    rcutils_log_location_t location = { func, "ik_solver", static_cast<size_t>(line) };
    rcutils_log(&location, severity(level), logger_.get_name(), "%s", msg.c_str());
  }

private:
  static int severity(ik_solver::LogLevel level)
  {
    switch (level)
    {
      case ik_solver::LogLevel::Trace:
      case ik_solver::LogLevel::Debug:
        return RCUTILS_LOG_SEVERITY_DEBUG;
      case ik_solver::LogLevel::Info:
        return RCUTILS_LOG_SEVERITY_INFO;
      case ik_solver::LogLevel::Warn:
        return RCUTILS_LOG_SEVERITY_WARN;
      case ik_solver::LogLevel::Error:
        return RCUTILS_LOG_SEVERITY_ERROR;
      default:
        return RCUTILS_LOG_SEVERITY_FATAL;
    }
  }

  rclcpp::Logger logger_;
};

/**
 * @brief Returns the ROS 2 parameters of `node` under `prefix` as a ParamMap.
 *
 * Parameters given to the node (e.g. with a --params-file) under `prefix` are
 * declared if they are not declared yet, so the node does not need to know
 * their names. Keys are relative to the prefix: "ur_ik_solver.base_frame" ->
 * "base_frame"; deeper levels keep the '.' separator. Boolean and byte arrays
 * are not supported by ParamMap and are skipped with a warning.
 *
 * Call it every time the parameters are needed to pick up the changes made
 * with `ros2 param set`.
 *
 * @param node the node.
 * @param prefix parameter prefix, without the trailing '.'; empty for all the parameters.
 */
inline ik_solver::ParamMap paramsFromNode(rclcpp::Node& node, const std::string& prefix)
{
  const std::string full_prefix = prefix.empty() ? "" : prefix + ".";

  // Declare the overrides under the prefix
  for (const auto& [name, value] : node.get_node_parameters_interface()->get_parameter_overrides())
  {
    if (name.compare(0, full_prefix.size(), full_prefix) == 0 && !node.has_parameter(name))
      node.declare_parameter(name, value);
  }

  ik_solver::ParamMap params;
  const auto list = node.list_parameters(prefix.empty() ? std::vector<std::string>{} : std::vector<std::string>{ prefix },
                                          rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);
  for (const std::string& name : list.names)
  {
    if (name.compare(0, full_prefix.size(), full_prefix) != 0)
      continue;
    const std::string key = name.substr(full_prefix.size());
    const rclcpp::Parameter p = node.get_parameter(name);
    switch (p.get_type())
    {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        params[key] = p.as_bool();
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        params[key] = static_cast<int64_t>(p.as_int());
        break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        params[key] = p.as_double();
        break;
      case rclcpp::ParameterType::PARAMETER_STRING:
        params[key] = p.as_string();
        break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
        params[key] = p.as_double_array();
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      {
        const std::vector<int64_t>& v = p.as_integer_array();
        params[key] = std::vector<int64_t>(v.begin(), v.end());
        break;
      }
      case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
        params[key] = p.as_string_array();
        break;
      case rclcpp::ParameterType::PARAMETER_NOT_SET:
        break;
      default:
        RCLCPP_WARN(node.get_logger(), "Parameter '%s' has a type not supported by ik_solver: skipped", name.c_str());
        break;
    }
  }
  return params;
}

/**
 * @copydoc paramsFromNode(rclcpp::Node&, const std::string&)
 */
inline ik_solver::ParamMap paramsFromNode(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  return paramsFromNode(*node, prefix);
}

}  // namespace ik_solver

#endif  // IK_SOLVER__ROS_PARAMS_HPP
