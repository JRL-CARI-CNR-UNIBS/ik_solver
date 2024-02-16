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
    fprintf(stderr, "[WARNING] seed names are wrong, skip seed");
  }
  else
  {
    for (const ik_solver_msgs::Configuration& seed : seeds)
    {
      Configuration s;
      s.resize(joint_names.size());
      if (s.size() != (long)seed.configuration.size())
      {
        fprintf(stderr,"[WARNING] seed dimensions are wrong, given %zu, expected %zu. Skip it", seed.configuration.size(), s.size());
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

bool getTF(const std::string& a_name, const std::string& b_name, Eigen::Affine3d& T_ab)
{
#if ROS_VERSION == 1
  tf::StampedTransform location_transform;
  ros::Time t0 = ros::Time(0);
  std::string tf_error;
  tf::TransformListener listener;
  if (!listener.waitForTransform(a_name, b_name, t0, ros::Duration(10), ros::Duration(0.01), &tf_error))
  {
    printf("[WARNING] Unable to find a transform from %s to %s, tf error=%s", a_name.c_str(), b_name.c_str(), tf_error.c_str());
    return false;
  }

  try
  {
    listener.lookupTransform(a_name, b_name, t0, location_transform);
  }
  catch (...)
  {
    printf("[WARNING] Unable to find a transform from %s to %s, tf error=%s", a_name.c_str(), b_name.c_str(), tf_error.c_str());
    return false;
  }

  tf::poseTFToEigen(location_transform, T_ab);
  return true;
#elif ROS_VERSION == 2
  using namespace std::chrono_literals;
  geometry_msgs::msg::TransformStamped location_transform;
  tf2::TimePoint t0 = tf2::TimePointZero;
  std::string tf_error;
  tf2_ros::Buffer buffer(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  tf2_ros::TransformListener listener(buffer);
  try
  {
    location_transform = buffer.lookupTransform(a_name, b_name, t0, tf2::Duration(10s));
  }
  catch (...)
  {
    fprintf(stderr, "[WARNING] Unable to find a transform from %s to %s, tf error=%s", a_name.c_str(), b_name.c_str(), tf_error.c_str());
    return false;
  }

  T_ab = tf2::transformToEigen(location_transform);
  return true;
#endif
}
}
