/* Copyright (C) 2024 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */

#ifndef IK_SOLVER__INTERNAL__TYPES_H
#define IK_SOLVER__INTERNAL__TYPES_H

#include <cstddef>
#include <vector>
#include <Eigen/Core>

#include <ik_solver_msgs/Configuration.h>
#include <ik_solver_msgs/IkSolution.h>

namespace ik_solver
{
using Configuration = Eigen::VectorXd;
using Configurations = std::vector<Configuration>;
struct Solutions : std::tuple<ik_solver::Configurations, std::vector<double>, std::vector<double>, std::string>
{
  const ik_solver::Configurations& configurations() const {return std::get<0>(*this);}
  ik_solver::Configurations& configurations() {return std::get<0>(*this);}

  const std::vector<double>& translation_residuals() const {return std::get<1>(*this);}
  std::vector<double>& translation_residuals() {return std::get<1>(*this);}

  const std::vector<double>& rotation_residuals() const {return std::get<2>(*this);}
  std::vector<double>& rotation_residuals() {return std::get<2>(*this);}

  const std::string& message() const { return std::get<3>(*this); }
  std::string& message() { return std::get<3>(*this); }

  void clear() { configurations().clear(); translation_residuals().clear(); rotation_residuals().clear(); message().clear();}

};

ik_solver_msgs::Configuration& cast(ik_solver_msgs::Configuration& lhs, const ik_solver::Configuration& rhs);

ik_solver_msgs::Configuration cast(const ik_solver::Configuration& rhs);

ik_solver_msgs::Configuration& operator<<(ik_solver_msgs::Configuration& lhs, const ik_solver::Configuration& rhs);

ik_solver::Configuration& cast(ik_solver::Configuration& lhs, const ik_solver_msgs::Configuration& rhs);

ik_solver::Configuration cast(const ik_solver_msgs::Configuration& rhs);

ik_solver::Configuration& operator<<(ik_solver::Configuration& lhs, const ik_solver_msgs::Configuration& rhs);

ik_solver_msgs::IkSolution& cast(ik_solver_msgs::IkSolution& lhs, const ik_solver::Solutions& rhs);

ik_solver_msgs::IkSolution cast(const ik_solver::Solutions& rhs);

ik_solver::Solutions& cast(ik_solver::Solutions& lhs, const ik_solver_msgs::IkSolution& rhs);

ik_solver::Solutions cast(const ik_solver_msgs::IkSolution& rhs);

ik_solver::Solutions& operator<<(ik_solver::Solutions& lhs, const ik_solver_msgs::IkSolution& rhs);

std::vector<ik_solver_msgs::IkSolution>& cast(std::vector<ik_solver_msgs::IkSolution>& lhs,
                                              const std::vector<ik_solver::Solutions>& rhs);

std::vector<ik_solver_msgs::IkSolution> cast(const std::vector<ik_solver::Solutions>& rhs);

std::vector<ik_solver_msgs::IkSolution>& operator<<(std::vector<ik_solver_msgs::IkSolution>& lhs,
                                                     const std::vector<ik_solver::Solutions>& rhs);

}  // namespace ik_solver

#endif