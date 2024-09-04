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


#include "ik_solver_msgs/Configuration.h"
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>


// #include <ik_solver_core/ik_solver_base_class.h>
#include <ik_solver/internal/types.h>
#include <ik_solver/internal/utils.h>
#include <ik_solver/internal/services_1.h>
#include <ik_solver/internal/services_common.h>

#include <ik_solver_core/internal/SafeQueue.h>

namespace ik_solver
{
/**
 * @brief Construct a new Ik Services:: Ik Services object
 *
 * @param nh
 * @param ik_solvers
 */
IkServices::IkServices(ros::NodeHandle& nh, IkSolversPool& ik_solvers) : nh_(nh), IkServicesBase(ik_solvers)
{
  ik_server_ =          nh.advertiseService("get_ik", &IkServices::computeIK, this);
  ik_server_array_ =    nh.advertiseService("get_ik_array", &IkServices::computeIKArray, this);
  fk_server_ =          nh.advertiseService("get_fk", &IkServices::computeFK, this);
  fk_server_array_ =    nh.advertiseService("get_fk_array", &IkServices::computeFKArray, this);
  bound_server_array_ = nh.advertiseService("get_bounds", &IkServices::getBounds, this);
  frames_server_array_= nh.advertiseService("get_frames", &IkServices::getFrames, this);
  reconfigure_ =        nh.advertiseService("reconfigure", &IkServices::reconfigure, this);
  change_tool_ =        nh.advertiseService("change_tool", &IkServices::changeTool, this);
}

bool IkServices::computeIK(ik_solver_msgs::GetIk::Request& req, ik_solver_msgs::GetIk::Response& res)
{
  return IkServicesBase::computeIK(&req, &res);
}

bool IkServices::computeIKArray(ik_solver_msgs::GetIkArray::Request& req, ik_solver_msgs::GetIkArray::Response& res)
{
  return IkServicesBase::computeIKArray(&req, &res);
}

bool IkServices::computeFK(ik_solver_msgs::GetFk::Request& req, ik_solver_msgs::GetFk::Response& res)
{
  return IkServicesBase::computeFK(&req, &res);
}

bool IkServices::computeFKArray(ik_solver_msgs::GetFkArray::Request& req, ik_solver_msgs::GetFkArray::Response& res)
{
  return IkServicesBase::computeFKArray(&req, &res);
}

bool IkServices::getBounds(ik_solver_msgs::GetBound::Request& req, ik_solver_msgs::GetBound::Response& res)
{
  return IkServicesBase::getBounds(&req, &res);
}

bool IkServices::getFrames(ik_solver_msgs::GetFrames::Request& req, ik_solver_msgs::GetFrames::Response& res)
{
  return IkServicesBase::getFrames(&req, &res);
}

bool IkServices::reconfigure(Trigger::Request& req, Trigger::Response& res)
{
  return IkServicesBase::reconfigure(&req, &res);
}

bool IkServices::changeTool(ik_solver_msgs::ChangeTool::Request& req, ik_solver_msgs::ChangeTool::Response& res)
{
  IkServicesBase::changeTool(&req, &res);
  ROS_INFO("Change Tool Result: %d", res.result);
  return true;
}

}  // namespace ik_solver
