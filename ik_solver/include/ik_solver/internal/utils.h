#ifndef IK_SOLVER__INTERNAL__UITLS_H
#define IK_SOLVER__INTERNAL__UITLS_H

#include <ik_solver_core/internal/types.h>
#include <ik_solver_core/internal/utils.h>
#include <ik_solver_msgs/Configuration.h>

#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>


namespace ik_solver{

Configurations getSeeds(const std::vector<std::string>& joint_names, const std::vector<std::string>& seed_names, const std::vector<ik_solver_msgs::Configuration>& seeds);

inline std::string to_string(const geometry_msgs::Pose& mat)
{
  std::stringstream ss;
  ss << mat;
  std::string s = ss.str();
  std::regex_replace(s, std::regex("\\r\\n|\\r|\\n"), ", ");
  return s;
}

}

#endif //IK_SOLVER__INTERNAL__UITLS_H