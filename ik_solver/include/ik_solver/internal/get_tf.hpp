#ifndef IK_SOLVER__GET_TF_IMPL
#define IK_SOLVER__GET_TF_IMPL

#include <ik_solver/ik_solver.hpp>

namespace ik_solver
{
#if ROS_X == 1
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

bool IkSolver::getTF(const std::string& a_name, const std::string& b_name, Eigen::Affine3d& T_ab)
{
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
}

#elif ROS_X == 2

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
bool IkSolver::getTF(const std::string& a_name, const std::string& b_name, Eigen::Affine3d& T_ab) const
{
  using namespace std::chrono_literals;
  geometry_msgs::msg::TransformStamped location_transform;
  tf2::TimePoint t0 = tf2::TimePointZero;

  bool ret {true};
  try
  {
    location_transform = tf_buffer_->lookupTransform(a_name, b_name, t0, tf2::Duration(5s));
  }
  catch (tf2::LookupException ex)
  {
    fprintf(stderr, "[WARNING] Timeout: Unable to find a transform from %s to %s\n", a_name.c_str(), b_name.c_str());
    fprintf(stderr, "[WARNING] %s", ex.what());
    ret = false;
  }
  catch(std::exception ex)
  {
    fprintf(stderr, "[WARNING] Unable to find a transform from %s to %s\n", a_name.c_str(), b_name.c_str());
    fprintf(stderr, "[WARNING] %s", ex.what());
    ret = false;
  }

  if(ret)
  {
    T_ab = tf2::transformToEigen(location_transform);
  }

  return ret;
}

#endif

} // namespace ik_solver
#endif
