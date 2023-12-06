#include <ros/node_handle.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <cstddef>

#include <ik_solver_msgs/GetIkArray.h>
#include <ik_solver_ros/utils.h>


namespace ik_solver
{

std::string resolve_ns(const ros::NodeHandle& nh, const std::string& params_ns)
{
  std::string ret;
  bool is_private = params_ns.empty() ? false : params_ns[0] == '~';
  bool is_global = params_ns.empty() ? false : params_ns[0] == '/';
  bool is_relative = params_ns.empty() ? true : params_ns[0] == (!is_private && !is_global);

  if (is_private)
  {
    std::string _params_ns = params_ns;
    _params_ns[0] = '/';
    ret = ros::NodeHandle("~").getNamespace() + _params_ns;
  }
  else if (is_global)
  {
    ret = params_ns;
  }
  else
  {
    ret = nh.getNamespace() + "/" + params_ns;
  }

  ret = ret.back() == '/' ? ret : ret + "/";
  return ret;
}


bool getTF(const std::string& a_name, const std::string& b_name, Eigen::Affine3d& T_ab)
{
  tf::StampedTransform location_transform;
  ros::Time t0 = ros::Time(0);
  std::string tf_error;
  tf::TransformListener listener;
  if (!listener.waitForTransform(a_name, b_name, t0, ros::Duration(10), ros::Duration(0.01), &tf_error))
  {
    ROS_WARN("Unable to find a transform from %s to %s, tf error=%s", a_name.c_str(), b_name.c_str(), tf_error.c_str());
    return false;
  }

  try
  {
    listener.lookupTransform(a_name, b_name, t0, location_transform);
  }
  catch (...)
  {
    ROS_WARN("Unable to find a transform from %s to %s, tf error=%s", a_name.c_str(), b_name.c_str(), tf_error.c_str());
    return false;
  }

  tf::poseTFToEigen(location_transform, T_ab);
  return true;
}


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
    ROS_WARN("seed names are wrong, skip seed");
  }
  else
  {
    for (const ik_solver_msgs::Configuration& seed : seeds)
    {
      Configuration s;
      s.resize(joint_names.size());
      if (s.size() != (long)seed.configuration.size())
      {
        ROS_WARN("seed dimensions are wrong, given %zu, expected %zu. Skip it", seed.configuration.size(), s.size());
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
