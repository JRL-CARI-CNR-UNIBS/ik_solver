#include "ik_solver/internal/ik_solver_node.ros2.hpp"

#include <gtest/gtest.h>
#include <ros2_control_test_assets/descriptions.hpp>

// rclcpp must be initialized before creating the nodes (gtest_main does not do it)
class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};
static ::testing::Environment* const rclcpp_env = ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

TEST(IkSolverNode2Test, robot_description_from_param)
{
  std::string urdf = ros2_control_test_assets::minimal_robot_urdf;
  rclcpp::NodeOptions opt = rclcpp::NodeOptions()
                                .append_parameter_override("robot_description", urdf)
                                .append_parameter_override("type", "test_ik_solver/TestIkSolverPlugin");
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

  rclcpp::NodeOptions opt = rclcpp::NodeOptions()
                                .append_parameter_override("type", "test_ik_solver/TestIkSolverPlugin")
                                .append_parameter_override("robot_description_topic", "/robot_description");
  std::shared_ptr<ik_solver::IkSolverNode> ik_node = ik_solver::IkSolverNode::make_node("test_ik_solver_node", opt);
  EXPECT_FALSE(ik_node->ready());
  rclcpp::spin_some(ik_node->get_node_base_interface());
  dummy.get_clock()->sleep_for(std::chrono::seconds(1));
  EXPECT_TRUE(ik_node->ready());
}
