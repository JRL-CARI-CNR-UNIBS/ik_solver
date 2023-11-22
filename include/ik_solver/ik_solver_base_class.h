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

#ifndef IK_SOLVER__IKSOLVER_BASE_CLASS_H
#define IK_SOLVER__IKSOLVER_BASE_CLASS_H

#include <algorithm>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <cstddef>
#include <memory>
#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <cmath>
#include "Eigen/src/Core/Matrix.h"

#include <urdf/model.h>

#include <ik_solver/internal/types.h>

namespace ik_solver
{


class IkSolver
{
public:

  IkSolver() = default;
  IkSolver(const IkSolver&) = delete;
  IkSolver(const IkSolver&&) = delete;
  IkSolver(IkSolver&&) = delete;
  virtual ~IkSolver() = default;

  virtual bool config(const ros::NodeHandle& nh, const std::string& param_ns = "");
  
  // FK flange to base
  virtual Solutions getIk(const Eigen::Affine3d& T_base_flange, const Configurations& seeds, const int& desired_solutions = -1, const int& min_stall_iterations = -1, const int& max_stall_iterations = -1) = 0;

  // FK base to flange
  virtual Eigen::Affine3d getFK(const Configuration& s) = 0;

  const std::vector<std::string>& joint_names() const { return joint_names_; }
  const std::string& base_frame() const { return base_frame_; }
  const std::string& flange_frame() const { return flange_frame_; }
  const std::string& tool_frame() const { return tool_frame_; }
  const Eigen::Affine3d& transform_from_flange_to_tool() const { return T_tool_flange_; }
  Eigen::Affine3d transform_from_tool_to_flange() const { return T_tool_flange_.inverse(); }

  
  const JointsBoundaries& jb() const { return jb_; }
  const std::vector<bool>& revolute() const { return revolute_;}

  const int& min_stall_iterations() const { return min_stall_iter_; }
  const int& max_stall_iterations() const { return max_stall_iter_; }
  const int& desired_solutions() const { return desired_solutions_; }
  const int& parallelize() const { return parallelize_; }

  const std::string param_namespace() const {return params_ns_;}

protected:
  std::string params_ns_;
  ros::NodeHandle robot_nh_;
  
  Eigen::Affine3d T_tool_flange_;
  tf::TransformListener listener_;
  std::string base_frame_;
  std::string flange_frame_;
  std::string tool_frame_;
  std::vector<std::string> joint_names_;

  JointsBoundaries jb_;
  std::vector<bool> revolute_;
  int min_stall_iter_ = 998;
  int max_stall_iter_ = 999;
  int max_iter_ = 1000000;
  int desired_solutions_ = 8;
  int parallelize_ = 0;
  int exploit_solutions_as_seed_ = 0;
  
  urdf::Model model_;

  bool getFlangeTool();

};


}  //  namespace ik_solver

#include <ik_solver/internal/ik_solver_base_class_impl.h>

#endif  // IK_SOLVER__IKSOLVER_BASE_CLASS_H
