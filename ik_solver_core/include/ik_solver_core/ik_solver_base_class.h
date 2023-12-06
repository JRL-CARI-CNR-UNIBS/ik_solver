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

#ifndef IK_SOLVER__IK_SOLVER_CORE__IKSOLVER_BASE_CLASS_H
#define IK_SOLVER__IK_SOLVER_CORE__IKSOLVER_BASE_CLASS_H

#include <string>
#include <vector>
#include <memory>

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>

#include <urdf/model.h>

#include <ik_solver_core/types.h>

namespace ik_solver
{
struct IkSolverOptions
{
  virtual ~IkSolverOptions() = default;
  using Ptr = std::shared_ptr<IkSolverOptions>;
  using ConstPtr = std::shared_ptr<const IkSolverOptions>;

  std::string robot_description_xmlstring_;
  
  std::string base_frame_;
  std::string flange_frame_;
  std::string tool_frame_;

  int desired_solutions_;
  int min_stall_iter_;
  int max_stall_iter_;
  int parallelize_;

  std::vector<std::string> joint_names_; // All joint names, considering also the attached robot if present
  JointsBoundaries jb_;
  Eigen::Affine3d T_tool_flange_;

  std::shared_ptr<IkSolverOptions> base_robot_options_;
  std::shared_ptr<IkSolverOptions> attached_robot_options_;
};

using IkSolverOptionsPtr = IkSolverOptions::Ptr;
using IkSolverOptionsConstPtr = IkSolverOptions::ConstPtr;

/**
 * @brief
 *
 */
class IkSolver
{
public:
  IkSolver() = default;
  IkSolver(const IkSolver&) = delete;
  IkSolver(const IkSolver&&) = delete;
  IkSolver(IkSolver&&) = delete;
  virtual ~IkSolver() = default;

  virtual bool config(IkSolverOptionsConstPtr nh, std::string& what);

  // FK flange to base
  virtual Solutions getIk(const Eigen::Affine3d& T_base_flange, const Configurations& seeds,
                          const int& desired_solutions = -1, const int& min_stall_iterations = -1,
                          const int& max_stall_iterations = -1) = 0;

  // FK base to flange
  virtual Eigen::Affine3d getFK(const Configuration& s);

  const std::vector<std::string>& joint_names() const;
  const std::string& base_frame() const;
  const std::string& flange_frame() const;
  const std::string& tool_frame() const;
  const Eigen::Affine3d& transform_from_flange_to_tool() const;
  Eigen::Affine3d transform_from_tool_to_flange() const;
  const JointsBoundaries& jb() const;
  const std::vector<bool>& revolute() const;
  const int& min_stall_iterations() const;
  const int& max_stall_iterations() const;
  const int& desired_solutions() const;
  const int& parallelize() const;
  const std::string param_namespace() const;

protected:
  std::string params_ns_;

  Eigen::Affine3d T_tool_flange_;
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
};

}  //  namespace ik_solver

#endif  // IK_SOLVER__IK_SOLVER_CORE__IKSOLVER_BASE_CLASS_H
