#include "ik_solver/internal/ik_solver_node.ros2.hpp"

#include <gtest/gtest.h>
#include <ros2_control_test_assets/descriptions.hpp>
#include <cnr_param/cnr_param.h>

TEST(IkSolverNode2Test, robot_description_from_param)
{
  std::string urdf = ros2_control_test_assets::minimal_robot_urdf;
  rclcpp::NodeOptions opt = rclcpp::NodeOptions().append_parameter_override("robot_description", urdf).automatically_declare_parameters_from_overrides(true);
  std::string what;
  cnr::param::set("test_robot/type","ik_solver/TestIkSolver", what);
  std::shared_ptr<ik_solver::IkSolverNode> ik_node = ik_solver::IkSolverNode::make_node("test_ik_solver_node", opt);
  EXPECT_TRUE(ik_node->ready());
}

TEST(IkSolverNode2Test, robot_description_from_topic)
{
  std::string urdf = ros2_control_test_assets::minimal_robot_urdf;
  rclcpp::Node dummy = rclcpp::Node("dummy");
  auto pub_rd = dummy.create_publisher<std_msgs::msg::String>("/robot_description",rclcpp::QoS(1).transient_local());
  std_msgs::msg::String msg;
  msg.data = urdf;
  pub_rd->publish(msg);

  rclcpp::NodeOptions opt = rclcpp::NodeOptions();
  std::string what;
  cnr::param::set("test_robot/type","ik_solver/TestIkSolver", what);
  std::shared_ptr<ik_solver::IkSolverNode> ik_node = ik_solver::IkSolverNode::make_node("test_ik_solver_node", opt);
  EXPECT_FALSE(ik_node->ready());
  rclcpp::spin_some(ik_node->get_node_base_interface());
  dummy.get_clock()->sleep_for(std::chrono::seconds(1));
  EXPECT_TRUE(ik_node->ready());
}
