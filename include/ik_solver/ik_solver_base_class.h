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

#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Geometry>
#include <ik_solver_msgs/GetIk.h>
#include <ik_solver_msgs/GetIkArray.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <urdf/model.h>

namespace ik_solver
{
class IkSolver
{
public:

  bool config(const ros::NodeHandle& nh);

  bool computeIK(ik_solver_msgs::GetIk::Request& req,
                 ik_solver_msgs::GetIk::Response& res);

  bool computeIKArray( ik_solver_msgs::GetIkArray::Request& req,
                       ik_solver_msgs::GetIkArray::Response& res);

  virtual std::vector<Eigen::VectorXd> getIk(const Eigen::Affine3d& T_base_flange,
                                             const std::vector<Eigen::VectorXd> & seeds,
                                             const int& desired_solutions,
                                             const int& max_stall_iterations)=0;

protected:
  ros::NodeHandle nh_;
  ros::ServiceServer server_;
  ros::ServiceServer server_array_;

  Eigen::Affine3d T_tool_flange_;
  tf::TransformListener listener_;
  std::string base_frame_;
  std::string flange_frame_;
  std::string tool_frame_;
  std::vector<std::string> joint_names_;

  Eigen::VectorXd ub_;
  Eigen::VectorXd lb_;
  int max_stall_iter_=1000;
  int max_iter_=1000000;
  int desired_solutions_=8;

  urdf::Model model_;

  bool getFlangeTool();

  virtual bool customConfig()=0;

  bool outOfBound(const Eigen::VectorXd& c);

  bool getTF(const  std::string& a_name,
             const  std::string& b_name,
             Eigen::Affine3d& T_ab);

  std::vector<Eigen::VectorXd> getSeeds(const std::vector<std::string>& seed_names,
                                        const std::vector<ik_solver_msgs::Configuration>& seeds);

};
}  //  namespace ik_solver


#include <ik_solver/internal/ik_solver_base_class_impl.h>
