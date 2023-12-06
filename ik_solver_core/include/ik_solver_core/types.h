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
#include <string>
#include <eigen3/Eigen/Core>

namespace ik_solver
{
using Configuration = Eigen::VectorXd;
using Configurations = std::vector<Configuration>;

/**
 * @brief
 *
 */
struct Solutions : std::tuple<ik_solver::Configurations, std::vector<double>, std::vector<double>, std::string>
{
  const ik_solver::Configurations& configurations() const
  {
    return std::get<0>(*this);
  }
  ik_solver::Configurations& configurations()
  {
    return std::get<0>(*this);
  }

  const std::vector<double>& translation_residuals() const
  {
    return std::get<1>(*this);
  }
  std::vector<double>& translation_residuals()
  {
    return std::get<1>(*this);
  }

  const std::vector<double>& rotation_residuals() const
  {
    return std::get<2>(*this);
  }
  std::vector<double>& rotation_residuals()
  {
    return std::get<2>(*this);
  }

  const std::string& message() const
  {
    return std::get<3>(*this);
  }
  std::string& message()
  {
    return std::get<3>(*this);
  }

  void clear()
  {
    configurations().clear();
    translation_residuals().clear();
    rotation_residuals().clear();
    message().clear();
  }
};

/**
 * @brief
 *
 */
struct Range : std::pair<double, double>
{
  const double& min() const
  {
    return std::get<0>(*this);
  }
  double& min()
  {
    return std::get<0>(*this);
  }

  const double& max() const
  {
    return std::get<1>(*this);
  }
  double& max()
  {
    return std::get<1>(*this);
  }
};

/**
 * @brief
 *
 */
struct JointBoundaries : std::vector<Range>
{
  double lb() const
  {
    auto it = std::max_element(this->begin(), this->end(),
                               [](const Range& lhs, const Range& rhs) { return lhs.min() < rhs.min(); });
    return it == this->end() ? std::nan("1") : it->min();
  }
  double ub() const
  {
    auto it = std::min_element(this->begin(), this->end(),
                               [](const Range& lhs, const Range& rhs) { return lhs.max() < rhs.max(); });
    return it == this->end() ? std::nan("1") : it->max();
  }
};

/**
 * @brief
 *
 */
struct NamedJointBoundaries : std::pair<std::string, JointBoundaries>
{
  const std::string& jname() const
  {
    return this->first;
  }
  std::string& jname()
  {
    return this->first;
  }

  const JointBoundaries& jb() const
  {
    return this->second;
  }
  JointBoundaries& jb()
  {
    return this->second;
  }
};

/**
 * @brief
 *
 */
struct JointsBoundaries : std::vector<NamedJointBoundaries>
{
  Eigen::VectorXd lb() const
  {
    Eigen::VectorXd _lb(this->size());
    for (size_t i = 0; i < this->size(); i++)
    {
      _lb(i) = this->at(i).second.lb();
    }
    return _lb;
  }
  Eigen::VectorXd ub() const
  {
    Eigen::VectorXd _ub(this->size());
    for (size_t i = 0; i < this->size(); i++)
    {
      _ub(i) = this->at(i).second.ub();
    }
    return _ub;
  }
};

std::ostream& operator<<(std::ostream& stream, const Range& r);

std::ostream& operator<<(std::ostream& stream, const JointBoundaries& rr);

std::ostream& operator<<(std::ostream& stream, const NamedJointBoundaries& rr);

std::ostream& operator<<(std::ostream& stream, const JointsBoundaries& rr);

}  // namespace ik_solver

#endif
