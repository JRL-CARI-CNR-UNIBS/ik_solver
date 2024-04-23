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

#include <array>
#include <string>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <ik_solver/ik_solver_base_class.h>

#include <ik_solver/internal/services.h>

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

  // Creation of the ros1 node
  ros::init(argc, argv, "ik_server_node");
  ros::NodeHandle nh("~");

  // Load the IK solver plugin
  pluginlib::ClassLoader<ik_solver::IkSolver> ik_loader("ik_solver", "ik_solver::IkSolver");

  std::string plugin_name;
  if (!nh.getParam("type",plugin_name))
  {
    ROS_ERROR("%s/type is not defined",nh.getNamespace().c_str());
    return -1;
  }

  // Creation of the IKSolversPool.
  ik_solver::IkSolversPool  ik_solvers;
  ROS_DEBUG("Creating %s (type %s)",nh.getNamespace().c_str(),plugin_name.c_str());
  for(std::size_t i=0;i<ik_solver::MAX_NUM_PARALLEL_IK_SOLVER;i++ )
  {
    ik_solvers.at(i) = ik_loader.createInstance(plugin_name);
    ROS_DEBUG("Configuring %s (type %s)",nh.getNamespace().c_str(),plugin_name.c_str());
    if (!ik_solvers.at(i)->config(nh))
    {
      ROS_ERROR("unable to configure %s (type %s)",nh.getNamespace().c_str(),plugin_name.c_str());
      return 0;
    }
  }
  ROS_DEBUG("%s (type %s) is ready to compute IK",nh.getNamespace().c_str(),plugin_name.c_str());
  // ==============================================

  ik_solver::IkServices services(nh, ik_solvers);

  // ==============================================

  ros::spin();
  return 0;
}
