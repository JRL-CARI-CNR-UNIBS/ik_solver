#ifndef IK_SOLVER__IK_SOLVER_NODE_2
#define IK_SOLVER__IK_SOLVER_NODE_2

#include <array>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <std_msgs/msg/string.hpp>

// #include <ik_solver_core/ik_solver_base_class.h>
#include <ik_solver/ik_solver.hpp>

#include <ik_solver/internal/services.ros2.h>

namespace ik_solver
{
using namespace std::chrono_literals;

class IkSolverNode : public rclcpp::Node
{
private:
  IkSolverNode() = delete;

  IkSolverNode(const std::string& name, const rclcpp::NodeOptions& opt = rclcpp::NodeOptions()) :
      rclcpp::Node(name, opt),
      ik_loader_("ik_solver", "ik_solver::IkSolver"),
      server_ready_(false)
  {
    std::string what;

    if(!cnr::param::get(std::string(get_namespace()) + "/type", plugin_name_, what))
    {
      RCLCPP_ERROR(get_logger(), "%s/type is not defined", get_namespace());
      RCLCPP_DEBUG_STREAM(get_logger(), what);
      return;
    }

    std::string robot_description;
    // try obtaining robot_description from ros parameters
    this->declare_parameter("robot_description", rclcpp::PARAMETER_STRING);
    if(!this->get_parameter("robot_description", robot_description))
    {

      // If not from parameters, get robot_description from topic
      RCLCPP_INFO(this->get_logger(), "Recovering robot_description from topic");
      sub_robot_description_ = this->create_subscription<std_msgs::msg::String>("~/robot_description", rclcpp::QoS(1).transient_local().reliable(),
          std::bind(&IkSolverNode::configure_after_robot_description, this, std::placeholders::_1));
    }
    else
    {
      std_msgs::msg::String rd_msg;
      rd_msg.data = robot_description;
      configure_after_robot_description(rd_msg);
    }
  }

public:

  bool ready()
  {
    return server_ready_;
  }

  void setup_services_and_run(rclcpp::Executor& ex)
  {
    rclcpp::Node::SharedPtr s_ptr = shared_from_this();
    ik_solver::IkServices services(s_ptr, ik_solvers_);
    ex.add_node(this->get_node_base_interface());
    ex.spin();
    ex.remove_node(this->get_node_base_interface());
  }

  static
  std::shared_ptr<IkSolverNode> make_node(const std::string& name, const rclcpp::NodeOptions& opt = rclcpp::NodeOptions())
  {
    return std::shared_ptr<IkSolverNode>(new IkSolverNode(name, opt));
  }

protected:


  void configure_after_robot_description(const std_msgs::msg::String& msg)
  {
    auto robot_description = msg.data;

    if(!cnr::param::set(this->get_namespace()+std::string("/robot_description"), robot_description, param_what_))
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot set cnr::param(" << this->get_namespace() << std::string("/robot_description") << ") because: " << param_what_);
    }
    RCLCPP_DEBUG(this->get_logger(), "set robot_description as cnr::param");

    RCLCPP_INFO(get_logger(), "Creating %s (type %s)",get_namespace(), plugin_name_.c_str());
    for(std::size_t i=0;i<ik_solver::MAX_NUM_PARALLEL_IK_SOLVER;i++ )
    {
      ik_solvers_.at(i) = ik_loader_.createSharedInstance(plugin_name_);
      RCLCPP_DEBUG(get_logger(), "Configuring %s (type %s)",get_namespace(), plugin_name_.c_str());
      if (!ik_solvers_.at(i)->config(get_namespace()))
      {
        RCLCPP_ERROR(get_logger(), "unable to configure %s (type %s)",get_namespace(), plugin_name_.c_str());
        // return 0;
      }
    }

    RCLCPP_INFO(get_logger(), "%s (type %s) is ready to compute IK",get_namespace(),plugin_name_.c_str());
    server_ready_ = true;
  }
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_robot_description_;
  std::string param_what_;

  bool server_ready_;

  std::string plugin_name_;
  pluginlib::ClassLoader<ik_solver::IkSolver> ik_loader_;
  ik_solver::IkSolversPool  ik_solvers_;
};

} // namespace ik_solver

#endif
