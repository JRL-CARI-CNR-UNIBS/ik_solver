/**
 * @file utils.h
 * @brief ROS utilities of the IK solver node.
 */
#ifndef IK_SOLVER__INTERNAL__UITLS_H
#define IK_SOLVER__INTERNAL__UITLS_H

#include <ik_solver_core/internal/types.h>
#include <ik_solver_core/internal/utils.h>
#include "ik_solver/internal/types.h"

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace ik_solver{

/**
 * @brief Convert seed messages to configurations in solver joint order.
 * @param joint_names Joint names of the solver.
 * @param seed_names Joint names of the seeds.
 * @param seeds Seed messages.
 * @return Seeds in solver joint order.
 */
Configurations getSeeds(const std::vector<std::string>& joint_names, const std::vector<std::string>& seed_names, const std::vector<ik_solver_msgs::Configuration>& seeds);

/**
 * @brief String representation of a pose message.
 */
inline std::string to_string(const Pose& mat)
{
  std::string s;
  s = geometry_msgs::msg::to_yaml(mat);
  return s;
}

/**
 * @brief Convert a pose message to an Eigen transform.
 */
inline void from_pose_to_eigen(const Pose& pose, Eigen::Affine3d& eig)
{
  tf2::fromMsg(pose, eig);
}

/**
 * @brief Convert an Eigen transform to a pose message.
 */
inline void from_eigen_to_pose(const Eigen::Affine3d& eig, Pose& pose)
{
  pose = tf2::toMsg(eig);
}


}

#endif //IK_SOLVER__INTERNAL__UITLS_H
