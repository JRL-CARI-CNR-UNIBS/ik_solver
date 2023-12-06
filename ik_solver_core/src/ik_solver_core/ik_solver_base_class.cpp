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

#include <cassert>

#include <exception>
#include <ostream>
#include <sstream>
#include <cmath>
#include <functional>
#include <iostream>
#include <numeric>
#include <valarray>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <string>
#include <chrono>
#include <mutex>
#include <tuple>
#include <vector>
#include <regex>

#include <Eigen/Core>
#include <cstdio>

#include <geometry_msgs/Pose.h>

#include <ik_solver_core/types.h>
//#include <ik_solver/internal/utils.h>
#include <ik_solver_core/ik_solver_base_class.h>

using namespace std::chrono_literals;

namespace ik_solver
{
bool IkSolver::config(IkSolverOptionsConstPtr opts, [[maybe_unused]] std::string& what)
{
  base_frame_ = opts->base_frame_;
  flange_frame_ = opts->flange_frame_;
  tool_frame_ = opts->tool_frame_;

  desired_solutions_ = opts->desired_solutions_;
  min_stall_iter_ = opts->min_stall_iter_;
  max_stall_iter_ = opts->max_stall_iter_;
  parallelize_ = opts->parallelize_;

  joint_names_ = opts->joint_names_;
  jb_ = opts->jb_;

  T_tool_flange_ = opts->T_tool_flange_;

  return true;
}

Eigen::Affine3d IkSolver::getFK(const Configuration& s)
{
  assert(0);
  return Eigen::Affine3d::Identity();
};

const std::vector<std::string>& IkSolver::joint_names() const
{
  return joint_names_;
}
const std::string& IkSolver::base_frame() const
{
  return base_frame_;
}
const std::string& IkSolver::flange_frame() const
{
  return flange_frame_;
}
const std::string& IkSolver::tool_frame() const
{
  return tool_frame_;
}
const Eigen::Affine3d& IkSolver::transform_from_flange_to_tool() const
{
  return T_tool_flange_;
}
Eigen::Affine3d IkSolver::transform_from_tool_to_flange() const
{
  return T_tool_flange_.inverse();
}

const JointsBoundaries& IkSolver::jb() const
{
  return jb_;
}
const std::vector<bool>& IkSolver::revolute() const
{
  return revolute_;
}

const int& IkSolver::min_stall_iterations() const
{
  return min_stall_iter_;
}
const int& IkSolver::max_stall_iterations() const
{
  return max_stall_iter_;
}
const int& IkSolver::desired_solutions() const
{
  return desired_solutions_;
}
const int& IkSolver::parallelize() const
{
  return parallelize_;
}

const std::string IkSolver::param_namespace() const
{
  return params_ns_;
}

}  // end namespace ik_solver
