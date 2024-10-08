/* Copyright (C) 2024 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */

#ifndef IK_SOLVER__INTERNAL__SERVICES_COMMON_H
#define IK_SOLVER__INTERNAL__SERVICES_COMMON_H

#include <memory>
#include <vector>
#include <array>
#include <Eigen/Geometry>

#include <ik_solver/internal/types.h>

#include <ik_solver_core/internal/wsq.h>

#if ROS_X == 1
#include <ros/ros.h>
#include <ik_solver_msgs/GetIk.h>
#include <ik_solver_msgs/GetFk.h>
#include <ik_solver_msgs/GetIkArray.h>
#include <ik_solver_msgs/GetFkArray.h>
#include <ik_solver_msgs/GetBound.h>
#include <ik_solver_msgs/GetFrames.h>
#include <ik_solver_msgs/ChangeTool.h>
#elif ROS_X == 2
//#include <rclcpp/rclcpp.hpp>
#include <ik_solver_msgs/srv/get_ik.hpp>
#include <ik_solver_msgs/srv/get_fk.hpp>
#include <ik_solver_msgs/srv/get_ik_array.hpp>
#include <ik_solver_msgs/srv/get_fk_array.hpp>
#include <ik_solver_msgs/srv/get_bound.hpp>
#include <ik_solver_msgs/srv/get_frames.hpp>
#include <ik_solver_msgs/srv/change_tool.hpp>
namespace ik_solver_msgs{
  using namespace srv;
}
#endif

#include "ik_solver/ik_solver.hpp"

namespace ik_solver
{
using IkArgs = std::tuple<Eigen::Affine3d, ik_solver::Configurations, std::size_t, int, int>;

inline const Eigen::Affine3d& get_T_base_flange(const IkArgs& args)
{
  return std::get<0>(args);
}
inline const ik_solver::Configurations& get_seeds(const IkArgs& args)
{
  return std::get<1>(args);
}
inline const std::size_t& get_desired_solutions(const IkArgs& args)
{
  return std::get<2>(args);
}
inline const int& get_min_stall_iterations(const IkArgs& args)
{
  return std::get<3>(args);
}
inline const int& get_max_stall_iterations(const IkArgs& args)
{
  return std::get<4>(args);
}

// std::shared_ptr<ik_solver::ThreadPool> ik_pool;
#if defined(_MAX_NUM_PARALLEL_IK_SOLVER) && (_MAX_NUM_PARALLEL_IK_SOLVER!=0)
  #define STR_HELPER(x) #x
  #define STR(x) STR_HELPER(x)
  #pragma message "IK SOLVER _MAX_NUM_PARALLEL_IK_SOLVER: " STR(_MAX_NUM_PARALLEL_IK_SOLVER)
  #define NUM_MAX_AXES MAX_NUM_AXES
#else
  #define _MAX_NUM_PARALLEL_IK_SOLVER 10
#endif

constexpr static const size_t MAX_NUM_PARALLEL_IK_SOLVER = _MAX_NUM_PARALLEL_IK_SOLVER;

using IkSolversPool = std::array<std::shared_ptr<ik_solver::IkSolver>, ik_solver::MAX_NUM_PARALLEL_IK_SOLVER>;

class IkServicesBase
{
protected:

  const IkSolver& config() const;

  IkServicesBase(IkSolversPool& ik_solvers) : ik_solvers_(ik_solvers) {}

  IkSolversPool& ik_solvers_;
  
  bool computeTransformations(const std::string& tip_frame, 
                              const std::string& reference_frame,
                              Eigen::Affine3d& T_poses_base, 
                              Eigen::Affine3d& T_flange_tool, 
                              Eigen::Affine3d& T_tool_tip);
                              

  std::vector<ik_solver::Solutions> computeIKArrayMT(const std::vector<Eigen::Affine3d>& v_T_b_f,
                                                          const std::vector<ik_solver::Configurations>& vseeds,
                                                          size_t desired_solutions, int min_stall_iterations, int max_stall_iterations);

  std::vector<ik_solver::Solutions> computeIKArrayST(const std::vector<Eigen::Affine3d>& v_T_b_f,
                                                          const std::vector<ik_solver::Configurations>& vseeds,
                                                          size_t desired_solutions, int min_stall_iterations, int max_stall_iterations,
                                                          uint8_t update_recursively_seeds);

  bool computeIK(ik_solver_msgs::GetIk::Request* req, ik_solver_msgs::GetIk::Response* res);

  bool computeFK(ik_solver_msgs::GetFk::Request* req, ik_solver_msgs::GetFk::Response* res);

  bool computeIKArray(ik_solver_msgs::GetIkArray::Request* req, ik_solver_msgs::GetIkArray::Response* res);

  bool computeFKArray(ik_solver_msgs::GetFkArray::Request* req, ik_solver_msgs::GetFkArray::Response* res);

  bool getBounds(ik_solver_msgs::GetBound::Request* req, ik_solver_msgs::GetBound::Response* res);

  bool getFrames(ik_solver_msgs::srv::GetFrames::Request *req, ik_solver_msgs::srv::GetFrames::Response *res);

  bool reconfigure(Trigger::Request* req, Trigger::Response* res);

  void changeTool(ik_solver_msgs::ChangeTool::Request* req, ik_solver_msgs::ChangeTool::Response* res);
};

}  // namespace ik_solver
#endif  // IK_SOLVER__INTERNAL__SERVICES_H
