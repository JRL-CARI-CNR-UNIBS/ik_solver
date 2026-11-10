/**
 * @file ik_solver_node.ros2.hpp
 * @brief ROS 2 node that loads an IK solver plugin and offers the IK/FK services.
 */
#ifndef IK_SOLVER__IK_SOLVER_NODE_2
#define IK_SOLVER__IK_SOLVER_NODE_2

#include <array>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <std_msgs/msg/string.hpp>

// #include <ik_solver_core/ik_solver_base_class.h>
#include <ik_solver/ik_solver.hpp>
#include <ik_solver/ros_params.hpp>

#include <ik_solver/internal/services.ros2.h>

namespace ik_solver
{
using namespace std::chrono_literals;

/**
 * @brief ROS 2 node that loads an IK solver plugin and offers the IK/FK services.
 *
 * The plugin name is read from the ROS parameter @c type. The URDF is read from the ROS
 * parameter @c robot_description or, if missing, from the topic given by the parameter
 * @c robot_description_topic (default @c ~/robot_description). Then
 * MAX_NUM_PARALLEL_IK_SOLVER instances of the plugin are created and configured with all
 * the ROS parameters of the node (see IkSolverBase). Solver messages go to the child
 * logger @c <node>.ik_solver.
 *
 * Usage:
 * @code
 * auto node = ik_solver::IkSolverNode::make_node("ik_solver_node");
 * while (!node->ready())
 *   rclcpp::spin_some(node->get_node_base_interface());
 * rclcpp::executors::MultiThreadedExecutor executor;
 * node->setup_services_and_run(executor);
 * @endcode
 */
class IkSolverNode : public rclcpp::Node
{
private:
  IkSolverNode() = delete;

  IkSolverNode(const std::string& name, const rclcpp::NodeOptions& opt = rclcpp::NodeOptions()) :
      rclcpp::Node(name, opt),
      server_ready_(false),
      ik_loader_("ik_solver", "ik_solver::IkSolver")
  {

    RCLCPP_DEBUG(this->get_logger(), "Creating TF Buffer");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    listener_  = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    RCLCPP_DEBUG(this->get_logger(), "Created TF Buffer");

    plugin_name_ = this->declare_parameter<std::string>("type", "");
    if(plugin_name_.empty())
    {
      RCLCPP_ERROR(get_logger(), "parameter 'type' (IK solver plugin) is not defined");
      return;
    }

    // try obtaining robot_description from ros parameters
    // (an empty default: get_parameter() of an uninitialized parameter throws)
    robot_description_ = this->declare_parameter<std::string>("robot_description", "");
    if(robot_description_.empty())
    {

      // If not from parameters, get robot_description from topic
      const std::string robot_description_topic =
          this->declare_parameter<std::string>("robot_description_topic", "~/robot_description");

      RCLCPP_INFO(this->get_logger(), "Recovering robot_description from topic");
      sub_robot_description_ = this->create_subscription<std_msgs::msg::String>(robot_description_topic, rclcpp::QoS(1).transient_local().reliable(),
          std::bind(&IkSolverNode::configure_after_robot_description, this, std::placeholders::_1));
    }
    else
    {
      std_msgs::msg::String rd_msg;
      rd_msg.data = robot_description_;
      configure_after_robot_description(rd_msg);
    }
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;

public:

  /**
   * @brief True when the solvers are created and configured.
   */
  bool ready()
  {
    return server_ready_;
  }

  /**
   * @brief Advertise the services and spin the executor (blocking).
   * @param ex Executor; a multi-threaded executor allows concurrent requests.
   */
  void setup_services_and_run(rclcpp::Executor& ex)
  {
    rclcpp::Node::SharedPtr s_ptr = shared_from_this();
    ik_solver::IkServices services(s_ptr, ik_solvers_);
    ex.add_node(this->get_node_base_interface());
    ex.spin();
    ex.remove_node(this->get_node_base_interface());
  }

  static
  /**
   * @brief Create the node.
   * @param name Node name.
   * @param opt Node options.
   * @return The node.
   */
  std::shared_ptr<IkSolverNode> make_node(const std::string& name, const rclcpp::NodeOptions& opt = rclcpp::NodeOptions())
  {
    return std::shared_ptr<IkSolverNode>(new IkSolverNode(name, opt));
  }

protected:


  /**
   * @brief Store the URDF, then create and configure the solver pool.
   */
  void configure_after_robot_description(const std_msgs::msg::String& msg)
  {
    robot_description_ = msg.data;

    ParamMap params = solverParams();

    RCLCPP_INFO(get_logger(), "Creating %s (type %s)",get_namespace(), plugin_name_.c_str());
    for(std::size_t i=0;i<ik_solver::MAX_NUM_PARALLEL_IK_SOLVER;i++ )
    {
      ik_solvers_.at(i) = ik_loader_.createSharedInstance(plugin_name_);
      RCLCPP_DEBUG(get_logger(), "Configuring %s (type %s)",get_namespace(), plugin_name_.c_str());
      ik_solvers_.at(i)->setBuffer(tf_buffer_);
      ik_solvers_.at(i)->setLogger(std::make_shared<RclcppLogger>(this->get_logger().get_child("ik_solver")));
      if (!ik_solvers_.at(i)->config(params))
      {
        RCLCPP_ERROR(get_logger(), "unable to configure %s (type %s)",get_namespace(), plugin_name_.c_str());
        // return 0;
      }

    }

    RCLCPP_INFO(get_logger(), "%s (type %s) is ready to compute IK",get_namespace(),plugin_name_.c_str());
    server_ready_ = true;
  }
public:
  /**
   * @brief Parameters of the solvers: all the ROS parameters of the node, plus the URDF
   * (@c robot_description). Read again at each call, e.g. by the reconfigure service.
   */
  ParamMap solverParams()
  {
    ParamMap params = paramsFromNode(*this, "");
    params["robot_description"] = robot_description_;
    return params;
  }

protected:
  std::string robot_description_;

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_robot_description_;

  bool server_ready_;

  std::string plugin_name_;
  pluginlib::ClassLoader<ik_solver::IkSolver> ik_loader_;
  ik_solver::IkSolversPool  ik_solvers_;
};

} // namespace ik_solver

#endif
