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

#ifndef IK_SOLVER__INTERNAL__TYPES_H
#define IK_SOLVER__INTERNAL__TYPES_H

#include <cstddef>
#include <vector>
#include <Eigen/Core>

#if ROS_VERSION == 1
  #include <ik_solver_msgs/Configuration.h>
  #include <ik_solver_msgs/IkSolution.h>
  #include <geometry_msgs/Pose.h>
  #include <std_srvs/Trigger.h>
  using Pose = geometry_msgs::Pose;
  using Trigger = std_srvs::Trigger;
#elif ROS_VERSION == 2
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
