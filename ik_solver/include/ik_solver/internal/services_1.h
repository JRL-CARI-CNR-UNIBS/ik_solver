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

#ifndef IK_SOLVER__INTERNAL__SERVICES_H
#define IK_SOLVER__INTERNAL__SERVICES_H

#include <memory>
#include <vector>
#include <array>
#include <Eigen/Geometry>

#include <ik_solver/internal/types.h>

#include <ik_solver_core/internal/wsq.h>

#include <ros/ros.h>
#include <ik_solver_msgs/GetIk.h>
#include <ik_solver_msgs/GetFk.h>
#include <ik_solver_msgs/GetIkArray.h>
#include <ik_solver_msgs/GetFkArray.h>
#include <ik_solver_msgs/GetBound.h>

#include <ik_solver/internal/services_common.h>

#include <ik_solver_core/ik_solver_base_class.h>

namespace ik_solver
{

class IkServices : public IkServicesBase
{
private:
  ros::ServiceServer ik_server_;
  ros::ServiceServer ik_server_array_;
  ros::ServiceServer fk_server_;
  ros::ServiceServer fk_server_array_;
  ros::ServiceServer bound_server_array_;
  ros::ServiceServer reconfigure_;

  ros::NodeHandle nh_;

public:
  IkServices() = delete;
  IkServices(const IkServices&) = delete;
  IkServices(IkServices&&) = delete;
  IkServices(const IkServices&&) = delete;
  ~IkServices() = default;

  IkServices(ros::NodeHandle& nh, IkSolversPool& ik_solvers);

  bool computeIK(ik_solver_msgs::GetIk::Request& req, ik_solver_msgs::GetIk::Response& res);

  bool computeFK(ik_solver_msgs::GetFk::Request& req, ik_solver_msgs::GetFk::Response& res);

  bool computeIKArray(ik_solver_msgs::GetIkArray::Request& req, ik_solver_msgs::GetIkArray::Response& res);

  bool computeFKArray(ik_solver_msgs::GetFkArray::Request& req, ik_solver_msgs::GetFkArray::Response& res);

  bool getBounds(ik_solver_msgs::GetBound::Request& req, ik_solver_msgs::GetBound::Response& res);

  bool reconfigure(Trigger::Request& req, Trigger::Response& res);
};

}  // namespace ik_solver
#endif  // IK_SOLVER__INTERNAL__SERVICES_H
