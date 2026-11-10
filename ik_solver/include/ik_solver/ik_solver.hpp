/**
 * @file ik_solver.hpp
 * @brief ROS base class of the IK solver plugins.
 */
#ifndef IK_SOLVER__IK_SOLVER
#define IK_SOLVER__IK_SOLVER

#include "ik_solver_core/ik_solver_base_class.h"

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>

namespace ik_solver
{
/**
 * @brief Base class of the IK solver plugins, loaded with pluginlib.
 *
 * Adds to IkSolverBase the TF lookup through a tf2 buffer. A plugin derives from this
 * class, implements config(), getIk() and getFK(), and is exported with
 * @c PLUGINLIB_EXPORT_CLASS(my_ns::MySolver, ik_solver::IkSolver).
 */
class IkSolver : public IkSolverBase
{
protected:

tf2_ros::Buffer::SharedPtr tf_buffer_;  ///< TF buffer used by getTF().

public:
  IkSolver() : IkSolverBase() {}
  IkSolver(const IkSolver&) = delete;
  IkSolver(const IkSolver&&) = delete;
  IkSolver(IkSolver&&) = delete;
  virtual ~IkSolver() = default;

  /**
   * @brief Set the TF buffer. It must be called before config().
   */
  void setBuffer(const tf2_ros::Buffer::SharedPtr& buffer){tf_buffer_=buffer;}

  /**
   * @brief Get the pose of @p b_name in @p a_name from TF (up to 30 attempts of 1 s).
   * @param a_name Reference frame.
   * @param b_name Target frame.
   * @param[out] T_ab Pose of b in a.
   * @return False if the transform is not available or the buffer is not set.
   */
  virtual bool getTF(const std::string& a_name, const std::string& b_name, Eigen::Affine3d& T_ab) const;

};


#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
inline bool IkSolver::getTF(const std::string& a_name, const std::string& b_name, Eigen::Affine3d& T_ab) const
{
  using namespace std::chrono_literals;
  geometry_msgs::msg::TransformStamped location_transform;
  tf2::TimePoint t0 = tf2::TimePointZero;
  bool ret {false};

  if (!tf_buffer_)
  {
    fprintf(stderr, "[ERROR] TF buffer is not set, did you call setBuffer()?\n");
    return false;
  }

  // 30 trials of 1 s (0.9 s lookup + 0.1 s sleep): about 30 s in total
  for (size_t itrial=0;itrial<30;itrial++)
  {
    ret = true;
    try
    {
      location_transform = tf_buffer_->lookupTransform(a_name, b_name, t0, tf2::Duration(900ms));
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
    if (ret)
      break;
    std::this_thread::sleep_for(0.1s);
  }

  if(ret)
  {
    T_ab = tf2::transformToEigen(location_transform);
  }

  return ret;
}



} // namespace ik_solver

//#include <ik_solver/internal/get_tf.hpp>
#endif // IK_SOLVER__IK_SOLVER
