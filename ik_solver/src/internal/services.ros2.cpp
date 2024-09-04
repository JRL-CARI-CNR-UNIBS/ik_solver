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

#include <functional>
#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <exception>
#include <future>
#include <iostream>
#include <iterator>
#include <optional>
#include <Eigen/Geometry>
#include <thread>
#include <vector>

#include "ik_solver_msgs/msg/configuration.hpp"
#include <geometry_msgs/msg/pose.hpp>

// #include <ik_solver_core/ik_solver_base_class.h>
#include <ik_solver/internal/types.h>
#include <ik_solver/internal/utils.h>
#include <ik_solver/internal/services.ros2.h>

#include <ik_solver_core/internal/SafeQueue.h>

namespace ik_solver
{

//========================================================================================

/**
 * @brief Construct a new Ik Services:: Ik Services object
 *
 * @param nh
 * @param ik_solvers
 */
IkServices::IkServices(rclcpp::Node::SharedPtr& nh, IkSolversPool& ik_solvers) : nh_(nh), IkServicesBase(ik_solvers)
{
  using namespace std::placeholders;
  ik_server_ =          nh->create_service<ik_solver_msgs::srv::GetIk>     ("get_ik",       std::bind(&IkServices::computeIK     , this, _1, _2));
  ik_server_array_ =    nh->create_service<ik_solver_msgs::srv::GetIkArray>("get_ik_array", std::bind(&IkServices::computeIKArray, this, _1, _2));
  fk_server_ =          nh->create_service<ik_solver_msgs::srv::GetFk>     ("get_fk",       std::bind(&IkServices::computeFK     , this, _1, _2));
  fk_server_array_ =    nh->create_service<ik_solver_msgs::srv::GetFkArray>("get_fk_array", std::bind(&IkServices::computeFKArray, this, _1, _2));
  bound_server_array_ = nh->create_service<ik_solver_msgs::srv::GetBound>  ("get_bounds",   std::bind(&IkServices::getBounds     , this, _1, _2));
  frames_server_array_= nh->create_service<ik_solver_msgs::srv::GetFrames> ("get_frames",   std::bind(&IkServices::getFrames     , this, _1, _2));
  reconfigure_ =        nh->create_service<std_srvs::srv::Trigger>         ("reconfigure",  std::bind(&IkServices::reconfigure   , this, _1, _2));
  change_tool_ =        nh->create_service<ik_solver_msgs::srv::ChangeTool>("change_tool",  std::bind(&IkServices::changeTool    , this, _1, _2));
  RCLCPP_DEBUG(nh->get_logger(), "IkServices created");
}


bool IkServices::computeIK(const ik_solver_msgs::GetIk::Request::SharedPtr req, ik_solver_msgs::GetIk::Response::SharedPtr res)
{
  return IkServicesBase::computeIK(req.get(), res.get());
}

bool IkServices::computeIKArray(const ik_solver_msgs::GetIkArray::Request::SharedPtr req, ik_solver_msgs::GetIkArray::Response::SharedPtr res)
{
  return IkServicesBase::computeIKArray(req.get(), res.get());
}

bool IkServices::computeFK(const ik_solver_msgs::GetFk::Request::SharedPtr req, ik_solver_msgs::GetFk::Response::SharedPtr res)
{
  return IkServicesBase::computeFK(req.get(), res.get());
}

bool IkServices::computeFKArray(const ik_solver_msgs::GetFkArray::Request::SharedPtr req, ik_solver_msgs::GetFkArray::Response::SharedPtr res)
{
  return IkServicesBase::computeFKArray(req.get(), res.get());
}

bool IkServices::getBounds(const ik_solver_msgs::GetBound::Request::SharedPtr req, ik_solver_msgs::GetBound::Response::SharedPtr res)
{
  return IkServicesBase::getBounds(req.get(), res.get());
}

bool IkServices::getFrames(const ik_solver_msgs::GetFrames::Request::SharedPtr req, ik_solver_msgs::GetFrames::Response::SharedPtr res)
{
  return IkServicesBase::getFrames(req.get(), res.get());
}

bool IkServices::reconfigure(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res)
{
  return IkServicesBase::reconfigure(req.get(), res.get());
}

void IkServices::changeTool(ik_solver_msgs::ChangeTool::Request::SharedPtr req, ik_solver_msgs::ChangeTool::Response::SharedPtr res)
{
  IkServicesBase::changeTool(req.get(), res.get());
  RCLCPP_INFO(nh_->get_logger(), "Change Tool Result: %d", res->result);
  return;
}

}  // namespace ik_solver
