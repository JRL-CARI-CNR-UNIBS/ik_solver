#ifndef IK_SOLVER__INTERNAL__UITLS_H
#define IK_SOLVER__INTERNAL__UITLS_H

#include <ik_solver_core/internal/types.h>
#include <ik_solver_core/internal/utils.h>
#include <ik_solver_msgs/Configuration.h>

namespace ik_solver{

Configurations getSeeds(const std::vector<std::string>& joint_names, const std::vector<std::string>& seed_names, const std::vector<ik_solver_msgs::Configuration>& seeds);

}

#endif //IK_SOLVER__INTERNAL__UITLS_H