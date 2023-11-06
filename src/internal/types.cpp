#include <ik_solver/internal/types.h>

namespace ik_solver
{
ik_solver_msgs::Configuration& cast(ik_solver_msgs::Configuration& lhs, const ik_solver::Configuration& rhs)
{
  lhs.configuration.clear();
  lhs.configuration.resize(rhs.rows(), 0);
  for (size_t i = 0; i < rhs.size(); i++)
    lhs.configuration[i] = rhs(i);
  return lhs;
}

ik_solver_msgs::Configuration cast(const ik_solver::Configuration& rhs)
{
  ik_solver_msgs::Configuration lhs;
  return cast(lhs, rhs);
}

ik_solver_msgs::Configuration& operator<<(ik_solver_msgs::Configuration& lhs, const ik_solver::Configuration& rhs)
{
  return cast(lhs, rhs);
}

ik_solver::Configuration& cast(ik_solver::Configuration& lhs, const ik_solver_msgs::Configuration& rhs)
{
  lhs.resize(rhs.configuration.size());
  lhs.setZero();
  for (size_t i = 0; i < rhs.configuration.size(); i++)
    lhs[i] = rhs.configuration[i];
  return lhs;
}

ik_solver::Configuration cast(const ik_solver_msgs::Configuration& rhs)
{
  ik_solver::Configuration lhs;
  return cast(lhs, rhs);
}

ik_solver::Configuration& operator<<(ik_solver::Configuration& lhs, const ik_solver_msgs::Configuration& rhs)
{
  return cast(lhs, rhs);
}

ik_solver_msgs::IkSolution& cast(ik_solver_msgs::IkSolution& lhs, const ik_solver::Configurations& rhs)
{
  lhs.configurations.clear();
  for (const auto& rhs_c : rhs)
  {
    ik_solver_msgs::Configuration lhs_c;
    lhs_c << rhs_c;
    lhs.configurations.push_back(lhs_c);
  }
  return lhs;
}

ik_solver_msgs::IkSolution cast(const ik_solver::Configurations& rhs)
{
  ik_solver_msgs::IkSolution lhs;
  return cast(lhs, rhs);
}

ik_solver_msgs::IkSolution& operator<<(ik_solver_msgs::IkSolution& lhs, const ik_solver::Configurations& rhs)
{
  return cast(lhs, rhs);
}

ik_solver::Configurations& cast(ik_solver::Configurations& lhs, const ik_solver_msgs::IkSolution& rhs)
{
  lhs.clear();
  for (const auto& rhs_c : rhs.configurations)
  {
    Eigen::VectorXd lhs_c(rhs_c.configuration.size());
    lhs_c << rhs_c;
    lhs.push_back(lhs_c);
  }
  return lhs;
}

ik_solver::Configurations cast(const ik_solver_msgs::IkSolution& rhs)
{
  ik_solver::Configurations lhs;
  return cast(lhs, rhs);
}

ik_solver::Configurations& operator<<(ik_solver::Configurations& lhs, const ik_solver_msgs::IkSolution& rhs)
{
  return cast(lhs, rhs);
}

std::vector<ik_solver_msgs::IkSolution>& cast(std::vector<ik_solver_msgs::IkSolution>& lhs,
                                              const std::vector<ik_solver::Configurations>& rhs)
{
  lhs.clear();
  for (const auto& rhs_c : rhs)
  {
    ik_solver_msgs::IkSolution lhs_c;
    lhs_c << rhs_c;
    lhs.push_back(lhs_c);
  }
  return lhs;
}

std::vector<ik_solver_msgs::IkSolution> cast(const std::vector<ik_solver::Configurations>& rhs)
{
  std::vector<ik_solver_msgs::IkSolution> lhs;
  return cast(lhs, rhs);
}

std::vector<ik_solver_msgs::IkSolution>& operator<<(std::vector<ik_solver_msgs::IkSolution>& lhs,
                                                    const std::vector<ik_solver::Configurations>& rhs)
{
  return cast(lhs, rhs);
}

}  // namespace ik_solver