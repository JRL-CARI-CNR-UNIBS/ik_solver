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
#include <std_msgs/msg/string.hpp>

#include <ik_solver_core/ik_solver_base_class.h>

#include <ik_solver/internal/services_2.h>

using namespace std::chrono_literals;

// TODO: conversion into lifecycle node
int main(int argc, char **argv)
{
  constexpr char robot_description_use_parameter[] {"use_parameter_robot_description"};

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("ik_solver_node");
  pluginlib::ClassLoader<ik_solver::IkSolver> ik_loader("ik_solver", "ik_solver::IkSolver");

  std::string plugin_name;
  std::string what;
  if(!cnr::param::get(std::string(node->get_namespace()) + "/type", plugin_name, what))
  {
    RCLCPP_ERROR(node->get_logger(), "%s/type is not defined",node->get_namespace());
    RCLCPP_DEBUG_STREAM(node->get_logger(), what);
    return -1;
  }

  // source name of the robot description
  node->declare_parameter(robot_description_use_parameter, false);
  std::string robot_description;
  if(not node->get_parameter(robot_description_use_parameter).as_bool())
  { // Get robot_description from topic
    RCLCPP_INFO(node->get_logger(), "Recovering robot_description from topic");
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_robot_description = node->create_subscription<std_msgs::msg::String>("/robot_description", 1, [](const std_msgs::msg::String& msg){});
    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(sub_robot_description);
    rclcpp::WaitResultKind wait_result_kind;
    do{
      RCLCPP_DEBUG(node->get_logger(), "Waiting robot_description");
      wait_result_kind = wait_set.wait(1s).kind();
    } while(wait_result_kind != rclcpp::WaitResultKind::Ready);

    std_msgs::msg::String msg; rclcpp::MessageInfo msg_info;
    sub_robot_description->take(msg, msg_info);
    robot_description = msg.data;
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Recovering robot_description from parameter");
    node->declare_parameter("robot_description", rclcpp::PARAMETER_STRING);
    if(not node->has_parameter("robot_description"))
    {
      RCLCPP_FATAL(node->get_logger(), "robot_description parameter is missing. Cannot initialize IkSolver");
      return -1;
    }
    robot_description = node->get_parameter("robot_description").as_string();
  }


  RCLCPP_DEBUG(node->get_logger(), "/robot_description: %s", robot_description.c_str());
  cnr::param::set(node->get_namespace()+std::string("/robot_description"), robot_description, what);
  RCLCPP_DEBUG(node->get_logger(), "what: %s", what.c_str());

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
