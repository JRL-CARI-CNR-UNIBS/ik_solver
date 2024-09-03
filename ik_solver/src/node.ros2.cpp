/**
 * @file node.cpp
 * @brief This file contains the main function for the IK solver node.
 *
 * The IK solver node initializes the ROS node, loads the IK solver plugin, configures the solver, and provides IK services.
 * It uses the pluginlib library to dynamically load the IK solver plugin based on the specified plugin name.
 * The node can handle multiple parallel IK solvers and provides services for computing IK solutions.
 */
/* Copyright (C) 2024 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */

#include "ik_solver/internal/ik_solver_node.ros2.hpp"
#include <rclcpp/rclcpp.hpp>

/**
 * @brief The main function of the node.
 *
 * This function initializes the ROS node, loads the IK solver plugin, and configures the IK solvers.
 * It then creates an instance of the IkServices class and starts the ROS spin loop.
 *
 * @param argc The number of command-line arguments.
 * @param argv An array of command-line arguments.
 * @return int The exit code of the program.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // ik_solver::IkSolverNode ik_node = ik_solver::IkSolverNode("ik_solver_node");
  std::shared_ptr<ik_solver::IkSolverNode> ik_node = ik_solver::IkSolverNode::make_node("ik_solver_node");
  while(!ik_node->ready())
  {
    RCLCPP_INFO_THROTTLE(ik_node->get_logger(), *ik_node->get_clock(), 1000, "Waiting for configuration. Probably robot_description is missing");
    rclcpp::spin_some(ik_node->get_node_base_interface());
    ik_node->get_clock()->sleep_for(rclcpp::Duration::from_seconds(0.1));
  };
  RCLCPP_INFO(ik_node->get_logger(), "IK Solver node: READY");
  rclcpp::executors::MultiThreadedExecutor executor;
  ik_node->setup_services_and_run(executor);
  return 0;
}
