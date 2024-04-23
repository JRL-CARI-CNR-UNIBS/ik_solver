/* Copyright (C) 2024 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
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
#include <tf/transform_listener.h>

#include <Eigen/Core>
#include <cstdio>

#include <geometry_msgs/Pose.h>
#include <ik_solver_msgs/GetIkArray.h>
#include <ik_solver_msgs/GetBound.h>
#include <ik_solver/internal/utils.h>

#include <ik_solver/ik_solver_base_class.h>

using namespace std::chrono_literals;

namespace ik_solver
{
inline bool IkSolver::config(const ros::NodeHandle& nh, const std::string& params_ns)
{
  params_ns_ = ik_solver::resolve_ns(nh, params_ns);
  robot_nh_ = nh;

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
    { params_ns_ + "parallel_ik_mode", { &parallelize_, ik_solver_msgs::GetIkArray::Request::PARALLELIZE_DISABLE } },
  };

  if (!get_and_default(iparams))
  {
    return false;
  }

  if (!getFlangeTool())
  {
    ROS_ERROR("%s: no TF from flange and tool", params_ns_.c_str());
    return false;
  }

  model_.initParam("robot_description");

  auto pn = params_ns_ + "joint_names";
  if (!ros::param::get(pn, joint_names_))
  {
    ROS_ERROR("[IkSolver::config] %s is not specified", pn.c_str());
    return false;
  }

  lb_.resize(joint_names_.size());
  ub_.resize(joint_names_.size());
  revolute_.resize(joint_names_.size());
  std::map<std::string, urdf::JointSharedPtr> joint_models = model_.joints_;
  for (size_t iax = 0; iax < joint_names_.size(); iax++)
  {
    if (joint_models.count(joint_names_.at(iax)) == 0)
    {
      ROS_ERROR("%s: %s is not a valid joint name", params_ns_.c_str(), joint_names_.at(iax).c_str());
      return false;
    }
    const urdf::JointSharedPtr& jmodel = joint_models.at(joint_names_.at(iax));
    lb_(iax) = jmodel->limits->lower;
    ub_(iax) = jmodel->limits->upper;

    revolute_.at(iax) = jmodel->type == jmodel->REVOLUTE;

    double value;
    if (ros::param::get(params_ns_ + "limits/" + joint_names_.at(iax) + "/upper", value))
    {
      ub_(iax) = std::min(ub_(iax), value);
    }
    if (ros::param::get(params_ns_ + "limits/" + joint_names_.at(iax) + "/lower", value))
    {
      lb_(iax) = std::max(lb_(iax), value);
    }
    if (lb_(iax) > ub_(iax))
    {
      ROS_ERROR("%s: %s has wrong limits: lower=%f, upper=%f", params_ns_.c_str(), joint_names_.at(iax).c_str(),
                lb_(iax), ub_(iax));
      return false;
    }
  }

  return true;
}

inline const std::vector<std::string>& IkSolver::joint_names() const
{
  return joint_names_;
}
inline const std::string& IkSolver::base_frame() const
{
  return base_frame_;
}
inline const std::string& IkSolver::flange_frame() const
{
  return IkSolver::flange_frame_;
}
inline const std::string& IkSolver::tool_frame() const
{
  return tool_frame_;
}
inline const Eigen::Affine3d& IkSolver::transform_from_flange_to_tool() const
{
  return T_tool_flange_;
}
inline Eigen::Affine3d IkSolver::transform_from_tool_to_flange() const
{
  return T_tool_flange_.inverse();
}
inline const Configuration& IkSolver::lb() const
{
  return lb_;
}
inline const Configuration& IkSolver::ub() const
{
  return ub_;
}
inline const std::vector<bool>& IkSolver::revolute() const
{
  return revolute_;
}
inline const int& IkSolver::min_stall_iterations() const
{
  return min_stall_iter_;
}
inline const int& IkSolver::max_stall_iterations() const
{
  return max_stall_iter_;
}
inline const int& IkSolver::desired_solutions() const
{
  return desired_solutions_;
}
inline const int& IkSolver::parallelize() const
{
  return parallelize_;
}
inline const std::string IkSolver::param_namespace() const
{
  return params_ns_;
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
