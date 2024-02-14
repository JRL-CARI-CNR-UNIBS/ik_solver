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
#include <ik_solver/internal/services_common.h>

#include <ik_solver_msgs/srv/get_ik.hpp>
#include <ik_solver_msgs/srv/get_fk.hpp>
#include <ik_solver_msgs/srv/get_ik_array.hpp>
#include <ik_solver_msgs/srv/get_fk_array.hpp>
#include <ik_solver_msgs/srv/get_bound.hpp>

#include <rclcpp/rclcpp.hpp>

namespace ik_solver
{

class IkServices : public IkServicesBase
{
private:
  rclcpp::Service<ik_solver_msgs::srv::GetIk>::SharedPtr      ik_server_;
  rclcpp::Service<ik_solver_msgs::srv::GetIkArray>::SharedPtr ik_server_array_;
  rclcpp::Service<ik_solver_msgs::srv::GetFk>::SharedPtr      fk_server_;
  rclcpp::Service<ik_solver_msgs::srv::GetFkArray>::SharedPtr fk_server_array_;
  rclcpp::Service<ik_solver_msgs::srv::GetBound>::SharedPtr   bound_server_array_;
  rclcpp::Service<Trigger>::SharedPtr                         reconfigure_;

  rclcpp::Node::SharedPtr nh_;

public:
  IkServices() = delete;
  IkServices(const IkServices&) = delete;
  IkServices(IkServices&&) = delete;
  IkServices(const IkServices&&) = delete;
  ~IkServices() = default;

  IkServices(rclcpp::Node::SharedPtr& nh, IkSolversPool& ik_solvers);

  bool computeIK(const ik_solver_msgs::srv::GetIk::Request::SharedPtr req, ik_solver_msgs::srv::GetIk::Response::SharedPtr res);

  bool computeFK(const ik_solver_msgs::srv::GetFk::Request::SharedPtr req, ik_solver_msgs::srv::GetFk::Response::SharedPtr res);

  bool computeIKArray(const ik_solver_msgs::srv::GetIkArray::Request::SharedPtr req, ik_solver_msgs::srv::GetIkArray::Response::SharedPtr res);

  bool computeFKArray(const ik_solver_msgs::srv::GetFkArray::Request::SharedPtr req, ik_solver_msgs::srv::GetFkArray::Response::SharedPtr res);

  bool getBounds(const ik_solver_msgs::srv::GetBound::Request::SharedPtr req, ik_solver_msgs::srv::GetBound::Response::SharedPtr res);

  bool reconfigure(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res);
};

}  // namespace ik_solver
#endif  // IK_SOLVER__INTERNAL__SERVICES_H
