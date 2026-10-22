/* Copyright (C) 2024 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */

/**
 * @file services_common.h
 * @brief ROS-version-independent implementation of the IK/FK services.
 */
#ifndef IK_SOLVER__INTERNAL__SERVICES_COMMON_H
#define IK_SOLVER__INTERNAL__SERVICES_COMMON_H

#include <memory>
#include <vector>
#include <array>
#include <Eigen/Geometry>

#include <ik_solver/internal/types.h>

#include <ik_solver_core/internal/wsq.h>

#if ROS_X == 1
#include <ros/ros.h>
#include <ik_solver_msgs/GetIk.h>
#include <ik_solver_msgs/GetFk.h>
#include <ik_solver_msgs/GetIkArray.h>
#include <ik_solver_msgs/GetFkArray.h>
#include <ik_solver_msgs/GetBound.h>
#include <ik_solver_msgs/SetBound.h>
#include <ik_solver_msgs/GetFrames.h>
#include <ik_solver_msgs/ChangeTool.h>
#include <ik_solver_msgs/SetInitialConfiguration.h>
#elif ROS_X == 2
// #include <rclcpp/rclcpp.hpp>
#include <ik_solver_msgs/srv/get_ik.hpp>
#include <ik_solver_msgs/srv/get_fk.hpp>
#include <ik_solver_msgs/srv/get_ik_array.hpp>
#include <ik_solver_msgs/srv/get_fk_array.hpp>
#include <ik_solver_msgs/srv/get_bound.hpp>
#include <ik_solver_msgs/srv/set_bound.hpp>
#include <ik_solver_msgs/srv/get_frames.hpp>
#include <ik_solver_msgs/srv/change_tool.hpp>
#include <ik_solver_msgs/srv/set_initial_configuration.hpp>
namespace ik_solver_msgs
{
  using namespace srv;
}
#endif

#include "ik_solver/ik_solver.hpp"

namespace ik_solver
{
  /**
   * @brief Arguments of one IK job: (T_base_flange, seeds, desired solutions, min stall iterations, max stall iterations).
   */
  using IkArgs = std::tuple<Eigen::Affine3d, ik_solver::Configurations, std::size_t, int, int>;

  /**
   * @brief Flange pose of an IK job.
   */
  inline const Eigen::Affine3d &get_T_base_flange(const IkArgs &args)
  {
    return std::get<0>(args);
  }
  /**
   * @brief Seeds of an IK job.
   */
  inline const ik_solver::Configurations &get_seeds(const IkArgs &args)
  {
    return std::get<1>(args);
  }
  /**
   * @brief Desired solutions of an IK job.
   */
  inline const std::size_t &get_desired_solutions(const IkArgs &args)
  {
    return std::get<2>(args);
  }
  /**
   * @brief Minimum stall iterations of an IK job.
   */
  inline const int &get_min_stall_iterations(const IkArgs &args)
  {
    return std::get<3>(args);
  }
  /**
   * @brief Maximum stall iterations of an IK job.
   */
  inline const int &get_max_stall_iterations(const IkArgs &args)
  {
    return std::get<4>(args);
  }

// std::shared_ptr<ik_solver::ThreadPool> ik_pool;
#if defined(_MAX_NUM_PARALLEL_IK_SOLVER) && (_MAX_NUM_PARALLEL_IK_SOLVER != 0)
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#pragma message "IK SOLVER _MAX_NUM_PARALLEL_IK_SOLVER: " STR(_MAX_NUM_PARALLEL_IK_SOLVER)
#define NUM_MAX_AXES MAX_NUM_AXES
#else
#define _MAX_NUM_PARALLEL_IK_SOLVER 10
#endif

  /**
   * @brief Number of solver instances (and worker threads) of a node.
 *
 * Set at build time by CMake through _MAX_NUM_PARALLEL_IK_SOLVER: number of CPU cores minus 2
 * (10 on machines with 3 cores or less).
   */
  constexpr static const size_t MAX_NUM_PARALLEL_IK_SOLVER = _MAX_NUM_PARALLEL_IK_SOLVER;

  /**
   * @brief Pool of solver instances, one per worker thread.
   */
  using IkSolversPool = std::array<std::shared_ptr<ik_solver::IkSolver>, ik_solver::MAX_NUM_PARALLEL_IK_SOLVER>;

  /**
   * @brief Implementation of the IK/FK services, shared by the ROS 1 and ROS 2 nodes.
   *
   * Service requests express target poses as the pose of the tool (or tip) frame in
   * the frame given by the pose header; the services convert them to flange poses in
   * the base frame before calling the solvers.
   *
   * Parameters read under the solver namespace (besides those of IkSolverBase):
   * - @c update_recursively_seeds: default seed update mode of IK arrays;
   * - @c task_redundant/...: perturbations used by the task-redundant IK (see README);
   * - @c filter_duplicates (bool, default true): remove duplicated solutions in the task-redundant IK;
   * - @c initial_conf (list of double): reference configuration used to choose among duplicates.
   */
  class IkServicesBase
  {
  protected:
    /**
     * @brief First solver of the pool, used to read the configuration.
     */
    const IkSolver &config() const;

    /**
     * @brief Constructor.
     * @param ik_solvers Configured solver pool.
     */
    IkServicesBase(IkSolversPool &ik_solvers) : ik_solvers_(ik_solvers) {}

    IkSolversPool &ik_solvers_;  ///< Solver pool.

    /**
     * @brief Compute the transforms needed by the FK services.
     * @param tip_frame Frame whose pose is requested (empty: tool frame).
     * @param reference_frame Frame of the returned poses.
     * @param[out] T_poses_base Pose of the base in the reference frame.
     * @param[out] T_flange_tool Pose of the tool in the flange frame.
     * @param[out] T_tool_tip Pose of the tip in the tool frame.
     * @return False if a transform is missing.
     */
    bool computeTransformations(const std::string &tip_frame,
                                const std::string &reference_frame,
                                Eigen::Affine3d &T_poses_base,
                                Eigen::Affine3d &T_flange_tool,
                                Eigen::Affine3d &T_tool_tip);

    /**
     * @brief Solve many IK problems in parallel, one per worker thread.
     * @param v_T_b_f Flange poses in the base frame.
     * @param vseeds Seeds of each pose.
     * @param desired_solutions Number of solutions per pose.
     * @param min_stall_iterations Minimum stall iterations.
     * @param max_stall_iterations Maximum stall iterations.
     * @param num_threads Number of worker threads; 0 or more than MAX_NUM_PARALLEL_IK_SOLVER means
     * MAX_NUM_PARALLEL_IK_SOLVER. Never more threads than poses are started.
     * @return Solutions of each pose.
     */
    std::vector<ik_solver::Solutions> computeIKArrayMT(const std::vector<Eigen::Affine3d> &v_T_b_f,
                                                       const std::vector<ik_solver::Configurations> &vseeds,
                                                       size_t desired_solutions, int min_stall_iterations, int max_stall_iterations,
                                                       size_t num_threads = 0);

    /**
     * @brief Solve many IK problems sequentially. The solutions of each pose are added as seeds of the next one.
     * @param v_T_b_f Flange poses in the base frame.
     * @param vseeds Seeds of each pose.
     * @param desired_solutions Number of solutions per pose.
     * @param min_stall_iterations Minimum stall iterations.
     * @param max_stall_iterations Maximum stall iterations.
     * @param update_recursively_seeds Seed update mode.
     * @return Solutions of each pose.
     */
    std::vector<ik_solver::Solutions> computeIKArrayST(const std::vector<Eigen::Affine3d> &v_T_b_f,
                                                       const std::vector<ik_solver::Configurations> &vseeds,
                                                       size_t desired_solutions, int min_stall_iterations, int max_stall_iterations,
                                                       uint8_t update_recursively_seeds);

    /**
     * @brief GetIk service: IK of one tool pose.
     */
    bool computeIK(ik_solver_msgs::GetIk::Request *req, ik_solver_msgs::GetIk::Response *res);

    /**
     * @brief GetFk service: pose of the tip frame for one configuration.
     */
    bool computeFK(ik_solver_msgs::GetFk::Request *req, ik_solver_msgs::GetFk::Response *res);

    /**
     * @brief GetIkArray service: IK of many tool poses, sequential or parallel according to @c parallelize.
     */
    bool computeIKArray(ik_solver_msgs::GetIkArray::Request *req, ik_solver_msgs::GetIkArray::Response *res);

    /**
     * @brief Task-redundant IK: IK of each target pose perturbed by all the perturbations
     * of computeTaskRedundantPerturbations(). The solutions of all perturbations of a target are
     * merged and, if @c filter_duplicates is set, duplicated configurations are removed.
     */
    bool computeTaskRedundantIKArray(ik_solver_msgs::GetIkArray::Request *req, ik_solver_msgs::GetIkArray::Response *res);

    /**
     * @brief Build the pose perturbations from the @c task_redundant parameters (grid or random).
     */
    std::vector<Eigen::Affine3d> computeTaskRedundantPerturbations() const;

    /**
     * @brief GetFkArray service: poses of the tip frame for many configurations (parallel).
     */
    bool computeFKArray(ik_solver_msgs::GetFkArray::Request *req, ik_solver_msgs::GetFkArray::Response *res);

    /**
     * @brief GetBound service: joint limits.
     */
    bool getBounds(ik_solver_msgs::GetBound::Request *req, ik_solver_msgs::GetBound::Response *res);

    /**
     * @brief SetBound service: change the joint limits of all solvers (see ik_solver_msgs/SetBound).
     */
    bool setBounds(ik_solver_msgs::SetBound::Request *req, ik_solver_msgs::SetBound::Response *res);

    /**
     * @brief GetFrames service: base, flange and tool frames.
     */
    bool getFrames(ik_solver_msgs::srv::GetFrames::Request *req, ik_solver_msgs::srv::GetFrames::Response *res);

    /**
     * @brief Reconfigure service: reload the parameters of all solvers.
     */
    bool reconfigure(Trigger::Request *req, Trigger::Response *res);

    /**
     * @brief ChangeTool service: change the tool frame of all solvers.
     */
    void changeTool(ik_solver_msgs::ChangeTool::Request *req, ik_solver_msgs::ChangeTool::Response *res);

    bool filter_duplicates_ = true;  ///< Remove duplicated solutions in the task-redundant IK.
    std::vector<double> initial_conf_;  ///< Reference configuration used to choose among duplicates.
    bool initial_conf_set_manually_ = false;  ///< True if initial_conf_ was set by service or setter (parameters do not override it).

  public:
    /**
     * @brief Read @c filter_duplicates and @c initial_conf.
     */
    void loadTaskRedundantParams();
    /**
     * @brief Set the reference configuration used to choose among duplicates.
     */
    void setInitialConfiguration(const std::vector<double> &conf)
    {
      initial_conf_ = conf;
      initial_conf_set_manually_ = true;
    }
    /**
     * @brief Reference configuration.
     */
    const std::vector<double> &getInitialConfiguration() const { return initial_conf_; }
    /**
     * @brief Enable or disable duplicate filtering.
     */
    void setFilterDuplicates(bool filter) { filter_duplicates_ = filter; }
    /**
     * @brief True if duplicate filtering is enabled.
     */
    bool getFilterDuplicates() const { return filter_duplicates_; }
    /**
     * @brief SetInitialConfiguration service.
     */
    bool setInitialConfiguration(ik_solver_msgs::SetInitialConfiguration::Request *req, ik_solver_msgs::SetInitialConfiguration::Response *res);
  };

} // namespace ik_solver
#endif // IK_SOLVER__INTERNAL__SERVICES_H
