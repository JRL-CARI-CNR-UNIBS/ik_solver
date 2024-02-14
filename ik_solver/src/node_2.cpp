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
  if (!node->get_parameter("type",plugin_name))
  {
    RCLCPP_ERROR(node->get_logger(), "%s/type is not defined",node->get_namespace());
    return -1;
  }

  ik_solver::IkSolversPool  ik_solvers;
  RCLCPP_DEBUG(node->get_logger(), "Creating %s (type %s)",node->get_namespace(), plugin_name.c_str());
  for(std::size_t i=0;i<ik_solver::MAX_NUM_PARALLEL_IK_SOLVER;i++ )
  {
//    ik_solvers.at(i) = ik_loader.createInstance(plugin_name);
    ik_solvers.at(i) = ik_loader.createSharedInstance(plugin_name);
    RCLCPP_DEBUG(node->get_logger(), "Configuring %s (type %s)",node->get_namespace(), plugin_name.c_str());
    if (!ik_solvers.at(i)->config())
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
