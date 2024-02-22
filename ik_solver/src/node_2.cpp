/*
Copyright (c) 2022, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <array>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <ik_solver_core/ik_solver_base_class.h>

#include <ik_solver/internal/services_2.h>

// TODO: conversion into lifecycle node
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("ik_solver_node");
//  ros::NodeHandle nh("~");
  pluginlib::ClassLoader<ik_solver::IkSolver> ik_loader("ik_solver", "ik_solver::IkSolver");

  std::string plugin_name;
  std::string what;
  if(!cnr::param::get(std::string(node->get_namespace()) + "/type", plugin_name, what))
  {
    RCLCPP_ERROR(node->get_logger(), "%s/type is not defined",node->get_namespace());
    RCLCPP_DEBUG_STREAM(node->get_logger(), what);
    return -1;
  }

  // Get robot_description
  node->declare_parameter("robot_description_holder", rclcpp::PARAMETER_STRING);
  const std::string robot_description_holder = node->get_parameter_or("robot_description_holder", std::string("/robot_state_publisher"));
  RCLCPP_DEBUG(node->get_logger(), "robot_description_holder: %s", robot_description_holder.c_str());
  rclcpp::SyncParametersClient::SharedPtr parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, robot_description_holder);
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
      return -1;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }
  if(!parameters_client->has_parameter("robot_description"))
  {
    RCLCPP_ERROR(node->get_logger(), "Node %s does not have the robot_description parameter", robot_description_holder.c_str());
    return -1;
  }
  const std::string robot_description = parameters_client->get_parameter<std::string>("robot_description");
  RCLCPP_DEBUG(node->get_logger(), "/robot_description: %s", robot_description.c_str());
  cnr::param::set(node->get_namespace()+std::string("/robot_description"), robot_description, what);
  RCLCPP_WARN(node->get_logger(), "what: %s", what.c_str());

  ik_solver::IkSolversPool  ik_solvers;
  RCLCPP_DEBUG(node->get_logger(), "Creating %s (type %s)",node->get_namespace(), plugin_name.c_str());
  for(std::size_t i=0;i<ik_solver::MAX_NUM_PARALLEL_IK_SOLVER;i++ )
  {
    ik_solvers.at(i) = ik_loader.createSharedInstance(plugin_name);
    RCLCPP_DEBUG(node->get_logger(), "Configuring %s (type %s)",node->get_namespace(), plugin_name.c_str());
    if (!ik_solvers.at(i)->config(node->get_namespace()))
    {
      RCLCPP_ERROR(node->get_logger(), "unable to configure %s (type %s)",node->get_namespace(), plugin_name.c_str());
      return 0;
    }
  }
  RCLCPP_DEBUG(node->get_logger(), "%s (type %s) is ready to compute IK",node->get_namespace(),plugin_name.c_str());
  // ==============================================

  ik_solver::IkServices services(node, ik_solvers);

  // ==============================================

  rclcpp::spin(node);
  return 0;
}
