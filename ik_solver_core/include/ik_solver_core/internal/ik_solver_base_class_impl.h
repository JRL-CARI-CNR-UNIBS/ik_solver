/* Copyright (C) 2024 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */

#ifndef IK_SOLVER__INTERNAL__IKSOLVER_BASE_CLASS_IMPL_H
#define IK_SOLVER__INTERNAL__IKSOLVER_BASE_CLASS_IMPL_H

#include <iostream>
#include <ostream>
#include <cstddef>
#include <sstream>
#include <algorithm>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <cstdio>
#include <cstdlib>
#include <string_view>

#include <ik_solver_core/ik_solver_base_class.h>

using namespace std::chrono_literals;

namespace ik_solver
{

constexpr static std::string_view ENV_LOGGER_CONFIG_PATH = "IK_SOLVER_LOGGER_CONFIG_PATH";

inline bool IkSolverBase::config(const std::string& params_ns)
{
  params_ns_ = ik_solver::resolve_ns(params_ns);

  std::string param_what;
  // TODO: check if exists the logger param file
  char* logger_config_path = std::getenv(ENV_LOGGER_CONFIG_PATH.data());

  if(logger_config_path == nullptr)
  {
    fprintf(stderr, "%s[ERROR]: Missing environemnt variable IK_SOLVER_LOGGER_CONFIG_PATH!%s\n", cnr_logger::BOLDRED().c_str(), cnr_logger::RESET().c_str());
    return false;
  }

  std::stringstream logger_id;
  logger_id << "ik_solver_" << reinterpret_cast<size_t>(this);
  if (!logger_.init(logger_id.str(), logger_config_path, false, false))
  {
    fprintf(stderr, "%s[ERROR]: Logger configuration failed: parameter file checked: %s%s\n", cnr_logger::BOLDRED().c_str(), logger_config_path, cnr_logger::RESET().c_str());
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
  CNR_DEBUG(logger_, "Solver parameters: OK");

  if (!getFlangeTool())
  {
    CNR_ERROR(logger_,"%s: no TF from flange and tool", params_ns_.c_str());
    return false;
  }
  CNR_DEBUG(logger_, "Flange->Tool transform: OK");


  if(!cnr::param::get(params_ns_ + "/robot_description", robot_description_, param_what))
  {
    CNR_ERROR(logger_, "IkSolver: Missing robot_description parameter\n%s",param_what.c_str());
    return false;
  }
  CNR_DEBUG(logger_, "robot_description from param: OK");

  // model_.initParam("robot_description");
  model_ = urdf::parseURDF(robot_description_);
  if(model_ == nullptr)
  {
    CNR_ERROR(logger_, "Cannot load robot_description!");
    return false;
  }
  CNR_DEBUG(logger_, "urdf model: OK");

  auto pn = params_ns_ + "joint_names";
  if (!cnr::param::get(pn, joint_names_, param_what))
  {
    CNR_DEBUG(logger_, "WHAT: %s", param_what.c_str());
    CNR_ERROR(logger_,"[IkSolverBase::config] %s is not specified", pn.c_str());
    return false;
  }
  CNR_DEBUG(logger_, "joint names from param: OK");

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

inline const std::vector<std::string>& IkSolverBase::joint_names() const
{
  return joint_names_;
}
inline const std::string& IkSolverBase::base_frame() const
{
  return base_frame_;
}
inline const std::string& IkSolverBase::flange_frame() const
{
  return IkSolverBase::flange_frame_;
}
inline const std::string& IkSolverBase::tool_frame() const
{
  return tool_frame_;
}
inline const Eigen::Affine3d& IkSolverBase::transform_from_flange_to_tool() const
{
  return T_tool_flange_;
}
inline Eigen::Affine3d IkSolverBase::transform_from_tool_to_flange() const
{
  return T_tool_flange_.inverse();
}
inline const Configuration& IkSolverBase::lb() const
{
  return lb_;
}
inline const Configuration& IkSolverBase::ub() const
{
  return ub_;
}
inline const std::vector<bool>& IkSolverBase::revolute() const
{
  return revolute_;
}
inline const int& IkSolverBase::min_stall_iterations() const
{
  return min_stall_iter_;
}
inline const int& IkSolverBase::max_stall_iterations() const
{
  return max_stall_iter_;
}
inline const int& IkSolverBase::desired_solutions() const
{
  return desired_solutions_;
}
inline const int& IkSolverBase::parallelize() const
{
  return parallelize_;
}
inline const std::string IkSolverBase::param_namespace() const
{
  return params_ns_;
}

inline bool IkSolverBase::getFlangeTool()
{
  return getTF(tool_frame_, flange_frame_, T_tool_flange_);
}

inline bool IkSolverBase::changeTool(const std::string &t_frame)
{
  Eigen::Affine3d T;
  if(!getTF(t_frame, flange_frame_, T))
  {
    CNR_ERROR(logger_, "Cannot change tool: missing transform from %s to %s", flange_frame_.c_str(), t_frame.c_str());
    return false;
  }
  T_tool_flange_ = T;
  tool_frame_ = t_frame;
  return true;
}

inline bool IkSolverBase::changeTool(const std::string &t_frame, const Eigen::Affine3d &T_tool_flange)
{
  CNR_DEBUG(logger_, "Changing tool without prior control of transformation");
  T_tool_flange_ = T_tool_flange;
  tool_frame_ = t_frame;
  return true;
}

// Eigen::Affine3d IkSolverBase::getFK(const Configuration& s)
// {
//   Eigen::Affine3d I;
//   I.setIdentity();
//   return I;
// }

}  // end namespace ik_solver

#endif  // IK_SOLVER__INTERNAL__IKSOLVER_BASE_CLASS_IMPL_H
