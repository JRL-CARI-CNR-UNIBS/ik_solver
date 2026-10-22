/* Copyright (C) 2024 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */

/**
 * @file ik_solver_base_class.h
 * @brief ROS-independent base class of the inverse kinematics solvers.
 */
#ifndef IK_SOLVER__IKSOLVER_BASE_CLASS_H
#define IK_SOLVER__IKSOLVER_BASE_CLASS_H

#include <string>

#include <cnr_logger/cnr_logger.h>
#include <cnr_param/cnr_param.h>

#include <Eigen/Geometry>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <ik_solver_core/internal/utils.h>
#include <ik_solver_core/internal/types.h>

/**
 * @brief Inverse kinematics solvers and related utilities.
 */
namespace ik_solver
{

/**
 * @brief ROS-independent base class of an inverse kinematics (IK) solver.
 *
 * A concrete solver implements getIk() and getFK() for a chain from @c base_frame to
 * @c flange_frame; the base class handles parameters, joint limits, the tool frame
 * (@c flange_frame → @c tool_frame transform) and the conversions between frames.
 * getTF() is implemented by the ROS layer (ik_solver::IkSolver).
 *
 * Frame naming: @c T_a_b (or @c T_ab) is the pose of frame b expressed in frame a.
 *
 * Parameters read by config() under the namespace @c param_ns:
 * - @c base_frame, @c flange_frame, @c tool_frame (string, required);
 * - @c joint_names (list of strings, required): joints of the chain, in solver order;
 * - @c robot_description (string, required): URDF, used to read the joint limits;
 * - @c limits/<joint>/upper, @c limits/<joint>/lower (double, optional): restrict the URDF limits
 *   (a value wider than the URDF one is ignored); they can be changed at runtime with setBounds();
 * - @c desired_solutions (int, default 8): number of solutions to look for;
 * - @c min_stall_iterations (int, default 998), @c max_stall_iterations (int, default 999):
 *   iterations without new solutions before stopping (meaning depends on the solver);
 * - @c parallel_ik_mode (int, default 2 = disabled): default parallelization mode of IK arrays.
 *
 * The environment variable @c IK_SOLVER_LOGGER_CONFIG_PATH must point to a cnr_logger configuration file.
 */
class IkSolverBase
{
public:

  IkSolverBase() = default;
  IkSolverBase(const IkSolverBase&) = delete;
  IkSolverBase(const IkSolverBase&&) = delete;
  IkSolverBase(IkSolverBase&&) = delete;
  virtual ~IkSolverBase() = default;

  /**
   * @brief Read the parameters and the joint limits (see class description).
   *
   * Derived classes overriding this function should call it first.
   * @param param_ns Parameter namespace.
   * @return False if a required parameter is missing, a joint is not in the URDF, or a transform is missing.
   */
  virtual bool config(const std::string& param_ns = "");

  // FK flange to base
  /**
   * @brief Compute the IK solutions of a flange pose. Implemented by each solver.
   * @param T_base_flange Pose of the flange in the base frame.
   * @param seeds Initial guesses (may be empty).
   * @param desired_solutions Number of solutions to look for (-1: use the parameter).
   * @param min_stall_iterations Minimum iterations without new solutions (-1: use the parameter).
   * @param max_stall_iterations Maximum iterations without new solutions (-1: use the parameter).
   * @return The solutions found (possibly none).
   */
  virtual Solutions getIk(const Eigen::Affine3d& T_base_flange, const Configurations& seeds,
                          const int& desired_solutions = -1, const int& min_stall_iterations = -1,
                          const int& max_stall_iterations = -1) = 0;

  // FK base to flange
  /**
   * @brief Compute the forward kinematics. Implemented by each solver.
   * @param s Joint configuration, in the order of joint_names().
   * @return Pose of the flange in the base frame.
   */
  virtual Eigen::Affine3d getFK(const Configuration& s) = 0;

  /**
   * @brief Compute the IK of a tool pose, given the frame transforms.
   * @param T_world_tool Pose of the tool in the world frame.
   * @param T_base_world Pose of the world in the base frame: the flange pose is T_base_world * T_world_tool * T_tool_flange.
   * @param T_tool_flange Pose of the flange in the tool frame.
   * @param seeds Initial guesses.
   * @param desired_solutions Number of solutions (-1: use the parameter).
   * @param min_stall_iterations Minimum stall iterations (-1: use the parameter).
   * @param max_stall_iterations Maximum stall iterations (-1: use the parameter).
   * @return The solutions.
   */
  Solutions computeIk(const Eigen::Affine3d& T_world_tool,
                      const Eigen::Affine3d& T_base_world,
                      const Eigen::Affine3d& T_tool_flange,
                      const Configurations& seeds,
                      const int& desired_solutions = -1,
                      const int& min_stall_iterations = -1,
                      const int& max_stall_iterations = -1);

  /**
   * @brief Compute the tool pose in the world frame, given the frame transforms.
   * @param s Joint configuration.
   * @param T_world_base Pose of the base in the world frame.
   * @param T_flange_tool Pose of the tool in the flange frame.
   * @return Pose of the tool in the world frame.
   */
  Eigen::Affine3d computeFk(const Configuration& s,
                            const Eigen::Affine3d& T_world_base,
                            const Eigen::Affine3d& T_flange_tool);

  /**
   * @brief Compute the IK of a tool pose; the transforms are read with getTF().
   * @param T_world_tool Pose of the tool in @p world_frame.
   * @param world_frame Frame of the pose.
   * @param tool_frame Tool frame.
   * @param seeds Initial guesses.
   * @param desired_solutions Number of solutions (-1: use the parameter).
   * @param min_stall_iterations Minimum stall iterations (-1: use the parameter).
   * @param max_stall_iterations Maximum stall iterations (-1: use the parameter).
   * @return The solutions (empty if a transform is missing).
   */
  Solutions computeIk(const Eigen::Affine3d& T_world_tool,
                      const std::string& world_frame,
                      const std::string& tool_frame,
                      const Configurations& seeds,
                      const int& desired_solutions = -1,
                      const int& min_stall_iterations = -1,
                      const int& max_stall_iterations = -1);

  /**
   * @brief Compute the pose of the current tool in @p world_frame; the transforms are read with getTF().
   * @param s Joint configuration.
   * @param world_frame Reference frame of the result.
   * @param base_frame Unused: the configured base frame is used.
   * @return Pose of the tool (uninitialized if a transform is missing).
   */
  Eigen::Affine3d computeFk(const Configuration& s,
                             const std::string& world_frame,
                             const std::string& base_frame);

  /**
   * @brief Joint names, in solver order.
   */
  const std::vector<std::string>& joint_names() const;
  /**
   * @brief Base frame of the kinematic chain.
   */
  const std::string& base_frame() const;
  /**
   * @brief Flange (last link) frame of the kinematic chain.
   */
  const std::string& flange_frame() const;
  /**
   * @brief Current tool frame.
   */
  const std::string& tool_frame() const;
  /**
   * @brief Pose of the flange in the tool frame (T_tool_flange).
   */
  const Eigen::Affine3d& transform_from_flange_to_tool() const;
  /**
   * @brief Pose of the tool in the flange frame (inverse of transform_from_flange_to_tool()).
   */
  Eigen::Affine3d transform_from_tool_to_flange() const;
  /**
   * @brief Lower joint limits.
   */
  const Configuration& lb() const;
  /**
   * @brief Upper joint limits.
   */
  const Configuration& ub() const;
  /**
   * @brief Set the joint limits, clamped to the URDF limits.
   *
   * The limits are restored from the parameters by config().
   * @param lb Lower limits (one per joint).
   * @param ub Upper limits (one per joint).
   * @return False (limits unchanged) if the sizes are wrong or, after clamping, a lower limit is greater than the upper one.
   */
  bool setBounds(const Configuration& lb, const Configuration& ub);
  /**
   * @brief True for revolute joints.
   */
  const std::vector<bool>& revolute() const;
  /**
   * @brief Default minimum stall iterations.
   */
  const int& min_stall_iterations() const;
  /**
   * @brief Default maximum stall iterations.
   */
  const int& max_stall_iterations() const;
  /**
   * @brief Default number of desired solutions.
   */
  const int& desired_solutions() const;
  /**
   * @brief Default parallelization mode of IK arrays (values of ik_solver_msgs/GetIkArray).
   */
  const int& parallelize() const;
  /**
   * @brief Parameter namespace (always ending with '/').
   */
  const std::string param_namespace() const;

  /**
   * @brief Change the tool frame, reading the flange-tool transform with getTF().
   * @param t_frame New tool frame.
   * @return False if the transform is not available.
   */
  bool changeTool(const std::string& t_frame);
  /**
   * @brief Change the tool frame using a given transform (not checked).
   * @param t_frame New tool frame.
   * @param T_tool_flange Pose of the flange in the new tool frame.
   * @return Always true.
   */
  bool changeTool(const std::string& t_frame, const Eigen::Affine3d& T_tool_flange);

  /**
   * @brief Get the pose of frame @p b_name in frame @p a_name. Implemented by the ROS layer.
   * @param a_name Reference frame.
   * @param b_name Target frame.
   * @param[out] T_ab Pose of b in a.
   * @return False if the transform is not available.
   */
  virtual bool getTF(const std::string& a_name, const std::string& b_name, Eigen::Affine3d& T_ab) const = 0;

protected:
  cnr_logger::TraceLogger logger_;  ///< Logger, configured from IK_SOLVER_LOGGER_CONFIG_PATH.
  bool logger_initialized_ = false;  ///< True after the first successful logger_.init().

  std::string params_ns_;  ///< Parameter namespace.
  std::string robot_description_;  ///< URDF.

  Eigen::Affine3d T_tool_flange_;  ///< Pose of the flange in the tool frame.
  std::string base_frame_;  ///< Base frame.
  std::string flange_frame_;  ///< Flange frame.
  std::string tool_frame_;  ///< Tool frame.
  std::vector<std::string> joint_names_;  ///< Joint names.

  Configuration ub_;  ///< Upper joint limits.
  Configuration lb_;  ///< Lower joint limits.
  Configuration urdf_ub_;  ///< Upper joint limits of the URDF.
  Configuration urdf_lb_;  ///< Lower joint limits of the URDF.
  std::vector<bool> revolute_;  ///< True for revolute joints.
  int min_stall_iter_ = 998;  ///< Default minimum stall iterations.
  int max_stall_iter_ = 999;  ///< Default maximum stall iterations.
  int max_iter_ = 1000000;  ///< Maximum iterations (available to solvers).
  int desired_solutions_ = 8;  ///< Default number of desired solutions.
  int parallelize_ = 0;  ///< Default parallelization mode.
  int exploit_solutions_as_seed_ = 0;  ///< Unused.

  constexpr static int PARALLELIZE_DISABLE = 2;  ///< Value of parallel_ik_mode that disables parallelization.

  urdf::ModelInterfaceSharedPtr model_;  ///< URDF model.

  /**
   * @brief Read the flange-tool transform with getTF().
   */
  bool getFlangeTool();


};


}  //  namespace ik_solver

#include <ik_solver_core/internal/ik_solver_base_class_impl.h>

#endif  // IK_SOLVER__IKSOLVER_BASE_CLASS_H
