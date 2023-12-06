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

#ifndef IK_SOLVER__INTERNAL__UITLS_H
#define IK_SOLVER__INTERNAL__UITLS_H

#include <string>
#include <vector>
#include <Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <ik_solver_core/types.h>

/**
 * @brief 
 * 
 */
namespace ik_solver
{

void printProgress(double percentage, const char* msg, ...);

bool is_the_same(const std::vector<std::string>& lhs, const std::vector<std::string>& rhs);

bool is_the_same(const std::string& lhs, const std::string& rhs);

bool isPresent(const Configuration& q, const Configurations& qq, double tolerance = 1e-4);

bool in_range(const double& v, const JointBoundaries& jb);

std::vector< std::pair<std::string, bool> > in_range(const Configuration& c, const JointsBoundaries& jb);

bool are_all_in_range(const Configuration& c, const JointsBoundaries& jb);

Configurations getMultiplicity(const Configurations& sol, const JointsBoundaries& jb, const std::vector<bool>& revolute);

void outOfBound(const Configuration& c, const JointsBoundaries& jb, std::vector<int>& ax_numbers);

void outOfBound(const Configuration& c, const JointsBoundaries& jb, std::vector<std::string>& ax_names);

void outOfBound(const Configuration& c, const JointsBoundaries& jb, std::vector<std::pair<std::string, int> >& ax_out_of_bound);

}



#endif  // IK_SOLVER__INTERNAL__UITLS_H
