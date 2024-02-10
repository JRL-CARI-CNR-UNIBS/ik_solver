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

#ifndef IK_SOLVER__INTERNAL__IKSOLVER_BASE_CLASS_IMPL_H
#define IK_SOLVER__INTERNAL__IKSOLVER_BASE_CLASS_IMPL_H

#include <cmath>
#include <functional>
#include <iostream>
#include <numeric>
#include <ostream>
#include <valarray>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <algorithm>
#include <string>
#include <chrono>
#include <mutex>
#include <tuple>
#include <vector>
#include <regex>

#include <Eigen/Core>
#include <cstdio>

#include <ik_solver_core/ik_solver_base_class.h>


using namespace std::chrono_literals;

namespace ik_solver
{

inline bool IkSolver::config(const std::string& params_ns)
{
  params_ns_ = ik_solver::resolve_ns(params_ns);

  std::string param_what;
  std::string logger_config_path;

  if(!cnr::param::get(params_ns_ + "logger_config_file", logger_config_path, param_what))
  {
    printf("[ERROR]: Missing parameter %s", (params_ns_ + "logger_config_file").c_str());
    return false;
  }
  // TODO: check if exists the logger param file
  // TODO: differentiate logger_id
  if (!logger_.init("ik_solver", logger_config_path, false, false))
  {
    printf("[ERROR]: Logger configuration failed");
    return false;
  }

  std::map<std::string, std::string*> sparams{
    { params_ns_ + "base_frame", &base_frame_ },
    { params_ns_ + "flange_frame", &flange_frame_ },
    { params_ns_ + "tool_frame", &tool_frame_ },
  };

  if (!get_and_return(sparams))
  {
    return false;
  }

  std::map<std::string, std::pair<int*, int>> iparams{
    { params_ns_ + "desired_solutions", { &desired_solutions_, desired_solutions_ } },
    { params_ns_ + "min_stall_iterations", { &min_stall_iter_, min_stall_iter_ } },
    { params_ns_ + "max_stall_iterations", { &max_stall_iter_, max_stall_iter_ } },
    // { params_ns_ + "parallel_ik_mode", { &parallelize_, ik_solver_msgs::GetIkArray::Request::PARALLELIZE_DISABLE } },
    { params_ns_ + "parallel_ik_mode", { &parallelize_, PARALLELIZE_DISABLE } },
  };

  if (!get_and_default(iparams))
  {
    return false;
  }

  if (!getFlangeTool())
  {
    CNR_ERROR(logger_,"%s: no TF from flange and tool", params_ns_.c_str());
    return false;
  }

  std::string robot_description;
  if(!cnr::param::get(params_ns_ + "robot_description", robot_description, param_what))
  {
    CNR_ERROR(logger_, "IkSolver: Missing robot_description parameter");
    return false;
  }
  
  // model_.initParam("robot_description");
  model_ = urdf::parseURDF(robot_description);

  auto pn = params_ns_ + "joint_names";
  if (!cnr::param::get(pn, joint_names_, param_what))
  {
    printf("WHAT: %s", param_what.c_str());
    CNR_ERROR(logger_,"[IkSolver::config] %s is not specified", pn.c_str());
    return false;
  }

  lb_.resize(joint_names_.size());
  ub_.resize(joint_names_.size());
  revolute_.resize(joint_names_.size());
  std::map<std::string, urdf::JointSharedPtr> joint_models = model_->joints_;
  for (size_t iax = 0; iax < joint_names_.size(); iax++)
  {
    if (joint_models.count(joint_names_.at(iax)) == 0)
    {
      CNR_ERROR(logger_,"%s: %s is not a valid joint name", params_ns_.c_str(), joint_names_.at(iax).c_str());
      return false;
    }
    const urdf::JointSharedPtr& jmodel = joint_models.at(joint_names_.at(iax));
    lb_(iax) = jmodel->limits->lower;
    ub_(iax) = jmodel->limits->upper;

    revolute_.at(iax) = jmodel->type == jmodel->REVOLUTE;

    double value;
    if (cnr::param::get(params_ns_ + "limits/" + joint_names_.at(iax) + "/upper", value, param_what))
    {
      ub_(iax) = std::min(ub_(iax), value);
    }
    if (cnr::param::get(params_ns_ + "limits/" + joint_names_.at(iax) + "/lower", value, param_what))
    {
      lb_(iax) = std::max(lb_(iax), value);
    }
    if (lb_(iax) > ub_(iax))
    {
      CNR_ERROR(logger_,"%s: %s has wrong limits: lower=%f, upper=%f", params_ns_.c_str(), joint_names_.at(iax).c_str(),
                lb_(iax), ub_(iax));
      return false;
    }
  }

  CNR_DEBUG(logger_, "IkSolver configured");
  return true;
}


inline bool IkSolver::getFlangeTool()
{
  return getTF(tool_frame_, flange_frame_, T_tool_flange_);
}

// Eigen::Affine3d IkSolver::getFK(const Configuration& s)
// {
//   Eigen::Affine3d I;
//   I.setIdentity();
//   return I;
// }

}  // end namespace ik_solver

#endif  // IK_SOLVER__INTERNAL__IKSOLVER_BASE_CLASS_IMPL_H
