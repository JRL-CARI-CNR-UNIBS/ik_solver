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

#include <vector>
#include <Eigen/Core>
#include <cstdio>
#include <ik_solver_msgs/GetBound.h>


#include <ik_solver/ik_solver_base_class.h>

namespace ik_solver
{

constexpr const char * PBSTR = "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||";
constexpr const size_t PBWIDTH = 60;

inline void printProgress(double percentage, const char* hdr, const char* msg)
{
  int val = (int)(percentage * 100);
  int lpad = (int)(percentage * PBWIDTH);
  int rpad = PBWIDTH - lpad;
  printf("\r[%s]%3d%% Ik Solver [%.*s%*s] %s", hdr, val, lpad, PBSTR, rpad, "", msg);
  fflush(stdout);
}

inline bool isPresent(const Eigen::VectorXd& q, const std::vector<Eigen::VectorXd>& vec)
{
  for (const Eigen::VectorXd& q2 : vec)
  {
    if ((q - q2).norm() < 1e-6)
      return true;
  }
  return false;
}

bool IkSolver::getBounds(ik_solver_msgs::GetBound::Request& req, ik_solver_msgs::GetBound::Response& res)
{
  res.joint_names.resize(joint_names_.size());
  res.lower_bound.resize(lb_.size());
  res.upper_bound.resize(ub_.size());

  for (size_t iax = 0; iax < lb_.size(); iax++)
  {
    res.joint_names.at(iax) = joint_names_.at(iax);
    res.lower_bound.at(iax) = lb_(iax);
    res.upper_bound.at(iax) = ub_(iax);
  }
  return true;
}

inline std::vector<Eigen::VectorXd> IkSolver::getMultiplicity(const std::vector<Eigen::VectorXd>& sol)
{
  std::vector<Eigen::VectorXd> multiturn_global;

  for (const Eigen::VectorXd& q : sol)
  {
    std::vector<Eigen::VectorXd> multiturn;
    std::vector<std::vector<double>> multiturn_ax(ub_.size());

    for (unsigned int idx = 0; idx < ub_.size(); idx++)
    {
      multiturn_ax.at(idx).push_back(q(idx));

      if (!revolute_.at(idx))
        continue;

      double tmp = q(idx);
      while (true)
      {
        tmp += 2 * M_PI;
        if (tmp > ub_(idx))
          break;
        multiturn_ax.at(idx).push_back(tmp);
      }
      tmp = q(idx);
      while (true)
      {
        tmp -= 2 * M_PI;
        if (tmp < lb_(idx))
          break;
        multiturn_ax.at(idx).push_back(tmp);
      }
    }

    if (!isPresent(q, multiturn))
      multiturn.push_back(q);

    for (unsigned int idx = 0; idx < ub_.size(); idx++)
    {
      size_t size_multiturn = multiturn.size();
      for (size_t is = 1; is < multiturn_ax.at(idx).size(); is++)
      {
        for (size_t im = 0; im < size_multiturn; im++)
        {
          Eigen::VectorXd new_q = multiturn.at(im);
          new_q(idx) = multiturn_ax.at(idx).at(is);
          if (!isPresent(new_q, multiturn))
            multiturn.push_back(new_q);
        }
      }
    }

    for (const Eigen::VectorXd& q2 : multiturn)
      if (not isPresent(q2, multiturn_global))
        multiturn_global.push_back(q2);
  }

  return multiturn_global;
}

inline bool IkSolver::config(const ros::NodeHandle& nh)
{
  nh_ = nh;

  if (!nh_.getParam("base_frame", base_frame_))
  {
    ROS_ERROR("%s/base_frame is not specified", nh_.getNamespace().c_str());
    return false;
  }
  if (!nh_.getParam("flange_frame", flange_frame_))
  {
    ROS_ERROR("%s/flange_frame is not specified", nh_.getNamespace().c_str());
    return false;
  }
  if (!nh_.getParam("tool_frame", tool_frame_))
  {
    ROS_ERROR("%s/tool_frame is not specified", nh_.getNamespace().c_str());
    return false;
  }
  if (!nh_.getParam("desired_solutions", desired_solutions_))
  {
    ROS_DEBUG("%s/desired_solutions is not specified, use 8", nh_.getNamespace().c_str());
    desired_solutions_ = 8;
  }

  if (!nh_.getParam("stall_iterations", max_stall_iter_))
  {
    ROS_DEBUG("%s/desired_solutions is not specified, use 8", nh_.getNamespace().c_str());
    max_stall_iter_ = 99;
  }

  if (not getFlangeTool())
  {
    ROS_ERROR("%s: no TF from flange and tool", nh_.getNamespace().c_str());
    return false;
  }

  model_.initParam("robot_description");

  if (!nh_.getParam("joint_names", joint_names_))
  {
    ROS_ERROR("%s/joint_names is not specified", nh_.getNamespace().c_str());
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
      ROS_ERROR("%s: %s is not a valid joint name", nh_.getNamespace().c_str(), joint_names_.at(iax).c_str());
      return false;
    }
    const urdf::JointSharedPtr& jmodel = joint_models.at(joint_names_.at(iax));
    lb_(iax) = jmodel->limits->lower;
    ub_(iax) = jmodel->limits->upper;

    revolute_.at(iax) = jmodel->type == jmodel->REVOLUTE;

    double value;
    if (nh_.getParam("limits/" + joint_names_.at(iax) + "/upper", value))
    {
      ub_(iax) = std::min(ub_(iax), value);
    }
    if (nh_.getParam("limits/" + joint_names_.at(iax) + "/lower", value))
    {
      lb_(iax) = std::max(lb_(iax), value);
    }
    if (lb_(iax) > ub_(iax))
    {
      ROS_ERROR("%s: %s has wrong limits: lower=%f, upper=%f", nh_.getNamespace().c_str(), joint_names_.at(iax).c_str(),
                lb_(iax), ub_(iax));
      return false;
    }
  }

  server_ = nh_.advertiseService("get_ik", &IkSolver::computeIK, this);
  server_array_ = nh_.advertiseService("get_ik_array", &IkSolver::computeIKArray, this);
  fk_server_array_ = nh_.advertiseService("get_fk_array", &IkSolver::computeFKArray, this);
  bound_server_array_ = nh_.advertiseService("get_bounds", &IkSolver::getBounds, this);
  return customConfig();
}

inline bool IkSolver::getFlangeTool()
{
  return getTF(tool_frame_, flange_frame_, T_tool_flange_);
}

inline bool IkSolver::computeIK(ik_solver_msgs::GetIk::Request& req, ik_solver_msgs::GetIk::Response& res)
{
  Eigen::Affine3d T_base_tool;
  if (not getTF(base_frame_, req.tf_name, T_base_tool))
    return false;

  Eigen::Affine3d T_base_flange = T_base_tool * T_tool_flange_;

  std::vector<Eigen::VectorXd> seeds = getSeeds(req.seed_joint_names, req.seeds);

  int desired_solutions = (req.max_number_of_solutions > 0) ? req.max_number_of_solutions : desired_solutions_;
  int max_stall_iterations = (req.stall_iterations > 0) ? req.stall_iterations : max_stall_iter_;
  std::vector<Eigen::VectorXd> solutions = getIk(T_base_flange, seeds, desired_solutions, max_stall_iterations);

  for (Eigen::VectorXd& s : solutions)
  {
    ik_solver_msgs::Configuration sol;
    sol.configuration.resize(s.size());
    for (long idx = 0; idx < s.size(); idx++)
      sol.configuration.at(idx) = s(idx);
    res.solution.configurations.push_back(sol);
  }
  res.joint_names = joint_names_;

  return true;
}

inline bool IkSolver::computeIKArray(ik_solver_msgs::GetIkArray::Request& req,
                                     ik_solver_msgs::GetIkArray::Response& res)
{
  Eigen::Affine3d T_base_poses;
  if (not getTF(base_frame_, req.poses.header.frame_id, T_base_poses))
    return false;

  std::vector<Eigen::VectorXd> seeds = getSeeds(req.seed_joint_names, req.seeds);

  int desired_solutions = (req.max_number_of_solutions > 0) ? req.max_number_of_solutions : desired_solutions_;
  int max_stall_iterations = (req.stall_iterations > 0) ? req.stall_iterations : max_stall_iter_;

  size_t failed_poses_counter = 0;
  for (size_t i = 0; i < req.poses.poses.size(); i++)
  {
    const geometry_msgs::Pose& p = req.poses.poses.at(i);
    ROS_DEBUG("computing IK for pose %zu of %zu", i, req.poses.poses.size());
    Eigen::Affine3d T_poses_tool;
    tf::poseMsgToEigen(p, T_poses_tool);

    Eigen::Affine3d T_base_flange = T_base_poses * T_poses_tool * T_tool_flange_;

    std::vector<Eigen::VectorXd> solutions = getIk(T_base_flange, seeds, desired_solutions, max_stall_iterations);

    ik_solver_msgs::IkSolution ik_sol;
    for (Eigen::VectorXd& s : solutions)
    {
      ik_solver_msgs::Configuration sol;
      sol.configuration.resize(s.size());
      for (long idx = 0; idx < s.size(); idx++)
      {
        sol.configuration.at(idx) = s(idx);
      }
      ik_sol.configurations.push_back(sol);
    }
    res.solutions.push_back(ik_sol);
    if (solutions.size())
    {
      seeds = solutions;
    }
    else
    {
      failed_poses_counter++;
    }

    char buffer[128]={0};  // maximum expected length of the float
    std::string nl = (i == req.poses.poses.size()-1 ? "\n" : "");
    std::snprintf(buffer, 128, "OK/FAILED/TOT %03zu/%03zu/%03zu (Last IK sols %02zu, des. %d, stall it. %d)%s",
      i+1 - failed_poses_counter, failed_poses_counter, req.poses.poses.size(), 
      solutions.size(),desired_solutions, max_stall_iterations, nl.c_str());
    
    printProgress(double(i+1) / double(req.poses.poses.size()), req.poses.header.frame_id.c_str(), buffer);
  }
  res.joint_names = joint_names_;

  return true;
}

inline bool IkSolver::computeFKArray(ik_solver_msgs::GetFkArray::Request& req,
                                     ik_solver_msgs::GetFkArray::Response& res)
{
  Eigen::Affine3d T_tool_tip;
  if (req.tip_frame.empty())
  {
    T_tool_tip.setIdentity();
  }
  else if (!getTF(tool_frame_, req.tip_frame, T_tool_tip))
  {
    ROS_ERROR("IkSolver::computeFKArray: error on computing TF from tool_name=%s, tip_frame=%s", tool_frame_.c_str(),
              req.tip_frame.c_str());
    return false;
  }

  Eigen::Affine3d T_poses_base;
  Eigen::Affine3d T_flange_tool = T_tool_flange_.inverse();
  if (not getTF(req.reference_frame, base_frame_, T_poses_base))
    return false;
  res.poses.header.frame_id = req.reference_frame;
  Eigen::VectorXd q(joint_names_.size());
  std::vector<int> order(joint_names_.size());

  for (int idx = 0; idx < joint_names_.size(); idx++)
  {
    bool found = false;
    for (int iax = 0; iax < req.joint_names.size(); iax++)
    {
      if (!req.joint_names.at(iax).compare(joint_names_.at(idx)))
      {
        found = true;
        // ROS_INFO("%s at  position %d",req.joint_names.at(iax).c_str(),iax);
        order.at(idx) = iax;
        break;
      }
    }
    if (!found)
    {
      ROS_ERROR("IkSolver::computeFKArray joint names are not correct");
      return false;
    }
  }

  for (const ik_solver_msgs::Configuration& s : req.configurations)
  {
    for (int idx = 0; idx < joint_names_.size(); idx++)
      q(idx) = s.configuration.at(order.at(idx));
    Eigen::Affine3d fk = T_poses_base * getFK(q) * T_flange_tool * T_tool_tip;
    geometry_msgs::Pose p;
    tf::poseEigenToMsg(fk, p);
    res.poses.poses.push_back(p);
  }
  return true;
}

Eigen::Affine3d IkSolver::getFK(const Eigen::VectorXd& s)
{
  Eigen::Affine3d I;
  I.setIdentity();
  return I;
}

inline bool IkSolver::getTF(const std::string& a_name, const std::string& b_name, Eigen::Affine3d& T_ab)
{
  tf::StampedTransform location_transform;
  ros::Time t0 = ros::Time(0);
  std::string tf_error;
  if (!listener_.waitForTransform(a_name, b_name, t0, ros::Duration(10), ros::Duration(0.01), &tf_error))
  {
    ROS_WARN("Unable to find a transform from %s to %s, tf error=%s", a_name.c_str(), b_name.c_str(), tf_error.c_str());
    return false;
  }

  try
  {
    listener_.lookupTransform(a_name, b_name, t0, location_transform);
  }
  catch (...)
  {
    ROS_WARN("Unable to find a transform from %s to %s, tf error=%s", a_name.c_str(), b_name.c_str(), tf_error.c_str());
    return false;
  }

  tf::poseTFToEigen(location_transform, T_ab);
  return true;
}

inline std::vector<Eigen::VectorXd> IkSolver::getSeeds(const std::vector<std::string>& seed_names,
                                                       const std::vector<ik_solver_msgs::Configuration>& seeds)
{
  std::vector<Eigen::VectorXd> seeds_eigen;
  std::vector<Eigen::VectorXd> solutions;

  bool seed_ok = true;
  if (seeds.size() > 0)
  {
    if (joint_names_.size() != seed_names.size())
      seed_ok = false;
    else
    {
      for (size_t ij = 0; ij < joint_names_.size(); ij++)
      {
        if (joint_names_.at(ij).compare(seed_names.at(ij)))  // compare return true if different
        {
          seed_ok = false;
          break;
        }
      }
    }
  }

  if (not seed_ok)
  {
    ROS_WARN("seed names are wrong, skip seed");
  }
  else
  {
    for (const ik_solver_msgs::Configuration& seed : seeds)
    {
      Eigen::VectorXd s(lb_.size());
      if (s.size() != (long)seed.configuration.size())
      {
        ROS_WARN("seed dimensions are wrong, given %zu, expected %zu. Skip it", seed.configuration.size(), s.size());
        continue;
      }
      for (size_t idx = 0; idx < seed.configuration.size(); idx++)
        s(idx) = seed.configuration.at(idx);
      seeds_eigen.push_back(s);
    }
  }
  return seeds_eigen;
}

inline bool IkSolver::outOfBound(const Eigen::VectorXd& c)
{
  bool out_of_bound = false;
  for (int iax = 0; iax < lb_.size(); iax++)
  {
    if ((c(iax) < lb_(iax)) || (c(iax) > ub_(iax)) || (std::isnan(c(iax))))
    {
      out_of_bound = true;
      break;
    }
  }
  return out_of_bound;
}

}  // end namespace ik_solver

#endif // IK_SOLVER__INTERNAL__IKSOLVER_BASE_CLASS_IMPL_H