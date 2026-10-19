/* Copyright (C) 2024 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */

/**
 * @file types.h
 * @brief Conversions between ik_solver types and ik_solver_msgs messages.
 */
#ifndef IK_SOLVER__INTERNAL__TYPES_H
#define IK_SOLVER__INTERNAL__TYPES_H

#include <cstddef>
#include <vector>
#include <Eigen/Core>

#if ROS_X == 1
  #include <ik_solver_msgs/Configuration.h>
  #include <ik_solver_msgs/IkSolution.h>
  #include <geometry_msgs/Pose.h>
  #include <std_srvs/Trigger.h>
  using Pose = geometry_msgs::Pose;
  using Trigger = std_srvs::Trigger;
#elif ROS_X == 2
  #include <ik_solver_msgs/msg/configuration.hpp>
  #include <ik_solver_msgs/msg/ik_solution.hpp>
  #include <geometry_msgs/msg/pose.hpp>
  #include <std_srvs/srv/trigger.hpp>
  namespace ik_solver_msgs{
    using namespace msg;
  }
  using Trigger = std_srvs::srv::Trigger;
  using Pose = geometry_msgs::msg::Pose;
#endif

#include <ik_solver_core/internal/types.h>


namespace ik_solver
{

/**
 * @brief Copy a configuration into a message.
 */
ik_solver_msgs::Configuration& cast(ik_solver_msgs::Configuration& lhs, const ik_solver::Configuration& rhs);

/**
 * @brief Convert a configuration to a message.
 */
ik_solver_msgs::Configuration cast(const ik_solver::Configuration& rhs);

ik_solver_msgs::Configuration& operator<<(ik_solver_msgs::Configuration& lhs, const ik_solver::Configuration& rhs);

/**
 * @brief Copy a message into a configuration.
 */
ik_solver::Configuration& cast(ik_solver::Configuration& lhs, const ik_solver_msgs::Configuration& rhs);

/**
 * @brief Convert a message to a configuration.
 */
ik_solver::Configuration cast(const ik_solver_msgs::Configuration& rhs);

ik_solver::Configuration& operator<<(ik_solver::Configuration& lhs, const ik_solver_msgs::Configuration& rhs);

/**
 * @brief Copy solutions into an IkSolution message.
 */
ik_solver_msgs::IkSolution& cast(ik_solver_msgs::IkSolution& lhs, const ik_solver::Solutions& rhs);

/**
 * @brief Convert solutions to an IkSolution message.
 */
ik_solver_msgs::IkSolution cast(const ik_solver::Solutions& rhs);

/**
 * @brief Copy an IkSolution message into solutions.
 */
ik_solver::Solutions& cast(ik_solver::Solutions& lhs, const ik_solver_msgs::IkSolution& rhs);

/**
 * @brief Convert an IkSolution message to solutions.
 */
ik_solver::Solutions cast(const ik_solver_msgs::IkSolution& rhs);

ik_solver::Solutions& operator<<(ik_solver::Solutions& lhs, const ik_solver_msgs::IkSolution& rhs);

std::vector<ik_solver_msgs::IkSolution>& cast(std::vector<ik_solver_msgs::IkSolution>& lhs,
                                              const std::vector<ik_solver::Solutions>& rhs);

/**
 * @brief Convert a list of solutions to messages.
 */
std::vector<ik_solver_msgs::IkSolution> cast(const std::vector<ik_solver::Solutions>& rhs);

std::vector<ik_solver_msgs::IkSolution>& operator<<(std::vector<ik_solver_msgs::IkSolution>& lhs,
                                                     const std::vector<ik_solver::Solutions>& rhs);

}  // namespace ik_solver

#endif
