#include "ik_solver/internal/utils.h"

namespace ik_solver{

Configurations getSeeds(const std::vector<std::string>& joint_names, const std::vector<std::string>& seed_names, const std::vector<ik_solver_msgs::Configuration>& seeds)
{
  Configurations seeds_eigen;
  Configurations solutions;

  bool seed_ok = true;
  if (seeds.size() > 0)
  {
    if (joint_names.size() != seed_names.size())
    {
      seed_ok = false;
    }
    else
    {
      for (size_t ij = 0; ij < joint_names.size(); ij++)
      {
        if (joint_names.at(ij).compare(seed_names.at(ij)))  // compare return true if different
        {
          seed_ok = false;
          break;
        }
      }
    }
  }

  if (!seed_ok)
  {
    printf("[WARNING] seed names are wrong, skip seed");
  }
  else
  {
    for (const ik_solver_msgs::Configuration& seed : seeds)
    {
      Configuration s;
      s.resize(joint_names.size());
      if (s.size() != (long)seed.configuration.size())
      {
        printf("[WARNING] seed dimensions are wrong, given %zu, expected %zu. Skip it", seed.configuration.size(), s.size());
        continue;
      }
      for (size_t idx = 0; idx < seed.configuration.size(); idx++)
      {
        s[idx] = seed.configuration.at(idx);
      }
      seeds_eigen.push_back(s);
    }
  }
  return seeds_eigen;
}
}