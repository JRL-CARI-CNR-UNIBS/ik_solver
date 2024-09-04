/* Copyright (C) 2024 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */

#ifndef IK_SOLVER__IKSOLVER_BASE_CLASS_H
#define IK_SOLVER__IKSOLVER_BASE_CLASS_H

#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Geometry>

#include <urdf/model.h>

#include <ik_solver/internal/types.h>

namespace ik_solver
{

/**
 * @class IkSolver
 * @brief Base class for inverse kinematics solvers.
 *
 * This class provides a base implementation for inverse kinematics solvers.
 * Derived classes can inherit from this class to implement specific inverse
 * kinematics algorithms.
 */
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

protected:
  std::string params_ns_;
  ros::NodeHandle robot_nh_;

  Eigen::Affine3d T_tool_flange_;
  tf::TransformListener listener_;
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

  urdf::Model model_;

  bool getFlangeTool();
};

}  //  namespace ik_solver

#include <ik_solver/internal/ik_solver_base_class_impl.h>

#endif  // IK_SOLVER__IKSOLVER_BASE_CLASS_H
