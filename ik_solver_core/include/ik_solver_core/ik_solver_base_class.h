/* Copyright (C) 2024 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */

#ifndef IK_SOLVER__IKSOLVER_BASE_CLASS_H
#define IK_SOLVER__IKSOLVER_BASE_CLASS_H

#include <string>

#include <cnr_logger/cnr_logger.h>
#include <cnr_param/cnr_param.h>

#include <Eigen/Geometry>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <ik_solver_core/internal/utils.h>
#include <ik_solver_core/internal/types.h>

namespace ik_solver
{

class IkSolverBase
{
public:

  IkSolverBase() = default;
  IkSolverBase(const IkSolverBase&) = delete;
  IkSolverBase(const IkSolverBase&&) = delete;
  IkSolverBase(IkSolverBase&&) = delete;
  virtual ~IkSolverBase() = default;

  virtual bool config(const std::string& param_ns = "");

  // FK flange to base
  virtual Solutions getIk(const Eigen::Affine3d& T_base_flange, const Configurations& seeds,
                          const int& desired_solutions = -1, const int& min_stall_iterations = -1,
                          const int& max_stall_iterations = -1) = 0;

  // FK base to flange
  virtual Eigen::Affine3d getFK(const Configuration& s) = 0;

  const std::vector<std::string>& joint_names() const;
  const std::string& base_frame() const;
  const std::string& flange_frame() const;
  const std::string& tool_frame() const;
  const Eigen::Affine3d& transform_from_flange_to_tool() const;
  Eigen::Affine3d transform_from_tool_to_flange() const;
  const Configuration& lb() const;
  const Configuration& ub() const;
  const std::vector<bool>& revolute() const;
  const int& min_stall_iterations() const;
  const int& max_stall_iterations() const;
  const int& desired_solutions() const;
  const int& parallelize() const;
  const std::string param_namespace() const;

  bool changeTool(const std::string& t_frame);
  bool changeTool(const std::string& t_frame, const Eigen::Affine3d& T_tool_flange);

protected:
  cnr_logger::TraceLogger logger_;

  std::string params_ns_;
  std::string robot_description_;

  Eigen::Affine3d T_tool_flange_;
  std::string base_frame_;
  std::string flange_frame_;
  std::string tool_frame_;
  std::vector<std::string> joint_names_;

  Configuration ub_;
  Configuration lb_;
  std::vector<bool> revolute_;
  int min_stall_iter_ = 998;
  int max_stall_iter_ = 999;
  int max_iter_ = 1000000;
  int desired_solutions_ = 8;
  int parallelize_ = 0;
  int exploit_solutions_as_seed_ = 0;

  constexpr static int PARALLELIZE_DISABLE = 2;

  urdf::ModelInterfaceSharedPtr model_;

  bool getFlangeTool();

  virtual bool getTF(const std::string& a_name, const std::string& b_name, Eigen::Affine3d& T_ab) const = 0;

};


}  //  namespace ik_solver

#include <ik_solver_core/internal/ik_solver_base_class_impl.h>

#endif  // IK_SOLVER__IKSOLVER_BASE_CLASS_H
