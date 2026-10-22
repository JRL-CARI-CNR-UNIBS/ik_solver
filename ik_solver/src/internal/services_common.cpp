/* Copyright (C) 2024 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <limits>
#include <mutex>
#include <cmath>
#include <functional>
#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <exception>
#include <future>
#include <iostream>
#include <iterator>
#include <optional>
#include <random>
#include <Eigen/Geometry>
#include <thread>
#include <vector>

// #include <ik_solver_core/ik_solver_base_class.h>
#include <ik_solver/internal/types.h>
#include <ik_solver/internal/utils.h>
#include <ik_solver/internal/services_common.h>

#include <ik_solver_core/internal/SafeQueue.h>

namespace ik_solver
{
  //========================================================================================
  /**
   * @brief The IkSolver->getIk solves the IK from 'flange' to 'base'. The IkArray
   *
   * @param solver
   * @param p_tool_in_base
   * @param T_base_tool
   * @param seeds
   * @param desired_solutions
   * @param max_stall_iterations
   * @return ik_solver::Configurations
   */
  ik_solver::Solutions computeIkFunction(ik_solver::IkSolver *solver, const Eigen::Affine3d &T_b_f,
                                         const ik_solver::Configurations &seeds, const int &desired_solutions,
                                         const int &min_stall_iterations, const int &max_stall_iterations)
  {
    ik_solver::Solutions solutions;
    solutions = solver->getIk(T_b_f, seeds, desired_solutions, min_stall_iterations, max_stall_iterations);
    return solutions;
  }

  //========================================================================================

  /**
   * @brief
   *
   * @param req
   * @param res
   * @return true
   * @return false
   */
  bool IkServicesBase::computeIK(ik_solver_msgs::GetIk::Request *req, ik_solver_msgs::GetIk::Response *res)
  {
    // The input pose is the TOOL pose
    // Frame {b}: robot base
    // Frame {r}: the frame id of the requested tool poses. It must be a solvable TF transform. No relative Transform.
    // TODO add check! Frame {t}: tool frame Frame {f}: flange frame
    Eigen::Affine3d T_b_r;

    if (!config().getTF(config().base_frame(), req->target.pose.header.frame_id, T_b_r))
    {
      res->success = false;
      res->message = "Failed in getting the TF from '" + config().base_frame() + "' to '" + req->target.pose.header.frame_id + "'";
      printf("Error: %s", res->message.c_str());
      return true;
    }

    Eigen::Affine3d T_r_t;
    from_pose_to_eigen(req->target.pose.pose, T_r_t);

    Eigen::Affine3d T_t_f = config().transform_from_flange_to_tool();
    Eigen::Affine3d T_b_f = T_b_r * T_r_t * T_t_f;

    Configurations seeds = ik_solver::getSeeds(config().joint_names(), req->seed_joint_names, req->target.seeds);

    int desired_solutions =
        (req->max_number_of_solutions > 0) ? req->max_number_of_solutions : config().desired_solutions();
    int min_stall_iterations = (req->stall_iterations > 0) ? req->stall_iterations : config().min_stall_iterations();
    int max_stall_iterations = (req->stall_iterations > 0) ? req->stall_iterations : config().max_stall_iterations();

    Solutions solutions = computeIkFunction(ik_solvers_.front().get(), T_b_f, seeds, desired_solutions,
                                            min_stall_iterations, max_stall_iterations);

    std::string msg = solutions.message().empty() ? "" : solutions.message();

    // printf(  "IK - Number of solutions %02zu, %s\n", solutions.configurations().size(), msg.c_str() );

    res->solution = ik_solver::cast(solutions);
    res->joint_names = config().joint_names();
    res->success = true;
    res->message = "OK";
    return true;
  }

  //========================================================================================
  /**
   * @brief
   *
   * @param req
   * @param res
   * @return true
   * @return false
   */
  bool IkServicesBase::computeIKArray(ik_solver_msgs::GetIkArray::Request *req, ik_solver_msgs::GetIkArray::Response *res)
  { // NOTE:
    // The input pose is the TOOL pose
    // Frame {b}: robot base
    // Frame {rp}: the frame id of the requested tool pose. It must be a solvable TF transform. No relative Transform.
    // TODO add check!

    // Frame {t}: tool frame
    // Frame {f}: flange frame
    //===============================
    std::vector<Eigen::Affine3d> poses_in_b;
    std::map<std::string, Eigen::Affine3d> T_b_rp;

    std::vector<ik_solver::Configurations> vseeds;
    std::vector<Eigen::Affine3d> v_T_b_f;
    for (const auto &t : req->targets)
    {
      Eigen::Affine3d T_b_r;
      if (T_b_rp.count(t.pose.header.frame_id) == 0)
      {
        if (!config().getTF(config().base_frame(), t.pose.header.frame_id, T_b_r))
        {
          return false;
        }
        T_b_rp[t.pose.header.frame_id] = T_b_r;
      }
      else
      {
        T_b_r = T_b_rp[t.pose.header.frame_id];
      }
      Eigen::Affine3d T_r_t;
      from_pose_to_eigen(t.pose.pose, T_r_t);

      Eigen::Affine3d T_t_f = config().transform_from_flange_to_tool();
      Eigen::Affine3d T_b_f = T_b_r * T_r_t * T_t_f;
      vseeds.push_back(ik_solver::getSeeds(config().joint_names(), req->seed_joint_names, t.seeds));
      v_T_b_f.push_back(T_b_f);
    }

    //===============================
    uint8_t parallelize = req->parallelize != ik_solver_msgs::GetIkArray::Request::PARALLELIZE_DEFAULT ? req->parallelize : config().parallelize();

    int update_recursively_seeds = ik_solver_msgs::GetIkArray::Request::UPDATE_RECURSIVELY_SEEDS_FORCE;
    std::string what;
    cnr::param::get(config().param_namespace() + "/update_recursively_seeds", update_recursively_seeds, what);
    update_recursively_seeds =
        req->update_recursively_seeds != ik_solver_msgs::GetIkArray::Request::UPDATE_RECURSIVELY_SEEDS_DEFAULT ? req->update_recursively_seeds : update_recursively_seeds;

    if (parallelize != ik_solver_msgs::GetIkArray::Request::PARALLELIZE_FORCE)
    {
      res->solutions =
          ik_solver::cast(computeIKArrayST(v_T_b_f, vseeds, config().desired_solutions(), config().min_stall_iterations(),
                                           config().max_stall_iterations(), update_recursively_seeds));
    }
    else
    {
      res->solutions = ik_solver::cast(computeIKArrayMT(v_T_b_f, vseeds, config().desired_solutions(),
                                                        config().min_stall_iterations(), config().max_stall_iterations(),
                                                        req->num_threads));
    }
    res->joint_names = config().joint_names();

    return true;
  }

  //========================================================================================
  namespace
  {
    std::vector<double> gridAxisValues(double min_v, double max_v, double step)
    {
      std::vector<double> values;
      if (step <= 0.0 || max_v <= min_v)
      {
        values.push_back(min_v);
        return values;
      }
      for (double v = min_v; v <= max_v + 1e-9; v += step)
      {
        values.push_back(v);
      }
      return values;
    }

    Eigen::Affine3d buildPerturbation(double dx, double dy, double dz, double roll, double pitch, double yaw)
    {
      return Eigen::Translation3d(dx, dy, dz) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    }
  } // namespace

  /**
   * @brief Reads the 'task_redundant' parameters (namespace under the solver's param namespace) and
   * builds the set of pose perturbations to be applied to each requested target. Two mutually exclusive
   * modes are supported, selected by the string parameter 'task_redundant/type':
   *  - "grid": for each axis (x,y,z,roll,pitch,yaw) reads min/max/step and builds the cartesian product.
   *  - "random": for each axis reads mean/standard_deviation and draws 'task_redundant/n_samples' samples,
   *    optionally seeded deterministically via 'task_redundant/seed' (falls back to a non-deterministic
   *    seed when unset).
   * An axis that is not configured defaults to a single perturbation value of 0 on that axis.
   */
  std::vector<Eigen::Affine3d> IkServicesBase::computeTaskRedundantPerturbations() const
  {
    std::string what;
    // "task_redundant" is the parameter name. The misspelled "task_reduntant" is still accepted
    // for backward compatibility with existing configuration files.
    std::string base = config().param_namespace();
    if (!base.empty() && base.back() != '/')
    {
      base += "/";
    }
    std::string ns = base + "task_redundant/";
    if (!cnr::param::has(base + "task_redundant", what) && cnr::param::has(base + "task_reduntant", what))
    {
      ns = base + "task_reduntant/";
    }

    std::string type = "grid";
    cnr::param::get(ns + "type", type, what);

    static const std::array<std::string, 6> axes = {"x", "y", "z", "roll", "pitch", "yaw"};

    std::vector<Eigen::Affine3d> perturbations;

    if (type == "random")
    {
      std::array<double, 6> mean{};
      std::array<double, 6> stddev{};
      for (size_t i = 0; i < axes.size(); i++)
      {
        cnr::param::get(ns + axes.at(i) + "/mean", mean.at(i), what);
        cnr::param::get(ns + axes.at(i) + "/standard_deviation", stddev.at(i), what);
      }

      int n_samples = 1;
      cnr::param::get(ns + "n_samples", n_samples, what);
      n_samples = std::max(n_samples, 1);

      // Optional deterministic seed (task_redundant/seed) for reproducible sampling, e.g. when
      // comparing Optuna trials that rely on Common Random Numbers. Falls back to a
      // non-deterministic seed, as before, when unset.
      int seed = 0;
      std::mt19937 gen;
      if (cnr::param::get(ns + "seed", seed, what))
      {
        gen.seed(static_cast<std::mt19937::result_type>(seed));
      }
      else
      {
        std::random_device rd;
        gen.seed(rd());
      }
      std::array<std::normal_distribution<double>, 6> dist;
      for (size_t i = 0; i < axes.size(); i++)
      {
        dist.at(i) = std::normal_distribution<double>(mean.at(i), stddev.at(i));
      }

      perturbations.reserve(static_cast<size_t>(n_samples));
      for (int s = 0; s < n_samples; s++)
      {
        std::array<double, 6> sample{};
        for (size_t i = 0; i < axes.size(); i++)
        {
          sample.at(i) = stddev.at(i) > 0.0 ? dist.at(i)(gen) : mean.at(i);
        }
        perturbations.push_back(
            buildPerturbation(sample[0], sample[1], sample[2], sample[3], sample[4], sample[5]));
      }
    }
    else
    {
      std::array<std::vector<double>, 6> values;
      for (size_t i = 0; i < axes.size(); i++)
      {
        double min_v = 0.0, max_v = 0.0, step = 1.0;
        cnr::param::get(ns + axes.at(i) + "/min", min_v, what);
        cnr::param::get(ns + axes.at(i) + "/max", max_v, what);
        cnr::param::get(ns + axes.at(i) + "/step", step, what);
        values.at(i) = gridAxisValues(min_v, max_v, step);
      }

      for (double x : values[0])
        for (double y : values[1])
          for (double z : values[2])
            for (double roll : values[3])
              for (double pitch : values[4])
                for (double yaw : values[5])
                {
                  perturbations.push_back(buildPerturbation(x, y, z, roll, pitch, yaw));
                }
    }

    return perturbations;
  }

  //========================================================================================
  /**
   * @brief Same request/response type as computeIKArray. Generates n pose*perturbation variants of the
   * request (see computeTaskRedundantPerturbations), solves each of them with computeIKArray and appends
   * the resulting solutions.
   *
   * @param req
   * @param res
   * @return true
   * @return false
   */
  namespace
  {

    inline double wrapTo2Pi(double angle)
    {
      double v = std::fmod(angle, 2.0 * M_PI);
      if (v < 0.0)
        v += 2.0 * M_PI;
      return v;
    }

    inline bool areConfigurationsEquivalent(const std::vector<double> &q1,
                                            const std::vector<double> &q2,
                                            const std::vector<bool> &revolute,
                                            double tolerance = 1e-3)
    {
      if (q1.size() != q2.size())
        return false;

      for (size_t i = 0; i < q1.size(); ++i)
      {
        bool is_rev = (i < revolute.size()) ? revolute[i] : true;
        double diff = q1[i] - q2[i];
        if (is_rev)
        {
          double diff_mod = std::remainder(diff, 2.0 * M_PI);
          if (std::abs(diff_mod) > tolerance)
            return false;
        }
        else
        {
          if (std::abs(diff) > tolerance)
            return false;
        }
      }
      return true;
    }

  } // namespace

  bool IkServicesBase::computeTaskRedundantIKArray(ik_solver_msgs::GetIkArray::Request *req,
                                                   ik_solver_msgs::GetIkArray::Response *res)
  {
    std::vector<Eigen::Affine3d> perturbations = computeTaskRedundantPerturbations();

    res->solutions.clear();
    res->solutions.resize(req->targets.size());
    res->joint_names = config().joint_names();

    if (req->targets.empty() || perturbations.empty())
    {
      return true;
    }

    // 1. Flatten all (perturbation x target) pairs into a single batch request
    ik_solver_msgs::GetIkArray::Request flat_req;
    flat_req.seed_joint_names = req->seed_joint_names;
    flat_req.parallelize = req->parallelize;
    flat_req.update_recursively_seeds = req->update_recursively_seeds;
    flat_req.num_threads = req->num_threads;
    flat_req.targets.reserve(perturbations.size() * req->targets.size());

    for (const auto &T_pert : perturbations)
    {
      for (const auto &t : req->targets)
      {
        auto t_pert = t;
        Eigen::Affine3d T_r_t;
        from_pose_to_eigen(t.pose.pose, T_r_t);
        T_r_t = T_r_t * T_pert;
        from_eigen_to_pose(T_r_t, t_pert.pose.pose);
        flat_req.targets.push_back(std::move(t_pert));
      }
    }

    // 2. Solve all P x N poses concurrently using all solver worker threads
    ik_solver_msgs::GetIkArray::Response flat_res;
    if (!computeIKArray(&flat_req, &flat_res))
    {
      return false;
    }

    // 3. Check duplicate filtering parameter
    loadTaskRedundantParams();

    if (!filter_duplicates_)
    {
      for (size_t p = 0; p < perturbations.size(); ++p)
      {
        for (size_t t_idx = 0; t_idx < req->targets.size(); ++t_idx)
        {
          size_t flat_idx = p * req->targets.size() + t_idx;
          const auto &sol = flat_res.solutions[flat_idx];

          if (sol.configurations.empty())
          {
            continue;
          }

          res->solutions[t_idx].configurations.insert(
              res->solutions[t_idx].configurations.end(),
              sol.configurations.begin(), sol.configurations.end());

          if (sol.translation_residual_errors.size() == sol.configurations.size())
          {
            res->solutions[t_idx].translation_residual_errors.insert(
                res->solutions[t_idx].translation_residual_errors.end(),
                sol.translation_residual_errors.begin(), sol.translation_residual_errors.end());
          }
          if (sol.rotation_residual_errors.size() == sol.configurations.size())
          {
            res->solutions[t_idx].rotation_residual_errors.insert(
                res->solutions[t_idx].rotation_residual_errors.end(),
                sol.rotation_residual_errors.begin(), sol.rotation_residual_errors.end());
          }
        }
      }

      flat_req.targets.clear();
      flat_req.targets.shrink_to_fit();
      flat_res.solutions.clear();
      flat_res.solutions.shrink_to_fit();

      return true;
    }

    // 3. Unflatten and accumulate unique solutions back into the respective targets,
    // keeping the duplicate configuration that is closest to initial_conf / seed
    const auto &revolute = config().revolute();
    constexpr double tolerance = 1e-3;
    constexpr double bucket_size = 0.05; // ~2.86 degrees
    const int num_buckets = static_cast<int>(std::ceil(2.0 * M_PI / bucket_size)) + 1;

    for (size_t t_idx = 0; t_idx < req->targets.size(); ++t_idx)
    {
      auto &target_sol = res->solutions[t_idx];
      std::vector<std::vector<size_t>> buckets(num_buckets);

      // Determine reference configuration for this target
      std::vector<double> ref_conf = initial_conf_;
      if (t_idx < req->targets.size() && !req->targets[t_idx].seeds.empty() &&
          !req->targets[t_idx].seeds[0].configuration.empty())
      {
        ref_conf = req->targets[t_idx].seeds[0].configuration;
      }

      auto computeDistSq = [&ref_conf](const std::vector<double> &q) -> double
      {
        if (ref_conf.empty())
          return 0.0;
        double d2 = 0.0;
        size_t n = std::min(q.size(), ref_conf.size());
        for (size_t i = 0; i < n; ++i)
        {
          double diff = q[i] - ref_conf[i];
          d2 += diff * diff;
        }
        return d2;
      };

      for (size_t p = 0; p < perturbations.size(); ++p)
      {
        size_t flat_idx = p * req->targets.size() + t_idx;
        const auto &sol = flat_res.solutions[flat_idx];

        if (sol.configurations.empty())
        {
          continue;
        }

        bool has_tra_err = (sol.translation_residual_errors.size() == sol.configurations.size());
        bool has_rot_err = (sol.rotation_residual_errors.size() == sol.configurations.size());

        for (size_t c_idx = 0; c_idx < sol.configurations.size(); ++c_idx)
        {
          const auto &cand_conf = sol.configurations[c_idx].configuration;
          if (cand_conf.empty())
          {
            continue;
          }

          // Bucket based on joint 0 to accelerate collision/duplicate searching
          double theta0 = (revolute.empty() || revolute[0]) ? wrapTo2Pi(cand_conf[0]) : cand_conf[0];
          int b = static_cast<int>(std::floor(theta0 / bucket_size)) % num_buckets;
          if (b < 0)
          {
            b += num_buckets;
          }

          int b_prev = (b - 1 + num_buckets) % num_buckets;
          int b_next = (b + 1) % num_buckets;

          bool is_duplicate = false;
          size_t matched_acc_idx = 0;
          const int check_buckets[3] = {b, b_prev, b_next};

          for (int check_b : check_buckets)
          {
            for (size_t acc_idx : buckets[check_b])
            {
              const auto &existing_conf = target_sol.configurations[acc_idx].configuration;
              if (areConfigurationsEquivalent(cand_conf, existing_conf, revolute, tolerance))
              {
                is_duplicate = true;
                matched_acc_idx = acc_idx;
                break;
              }
            }
            if (is_duplicate)
            {
              break;
            }
          }

          if (is_duplicate)
          {
            // If duplicate, keep whichever is closer to ref_conf (initial_conf)
            if (!ref_conf.empty())
            {
              double cand_d2 = computeDistSq(cand_conf);
              double exist_d2 = computeDistSq(target_sol.configurations[matched_acc_idx].configuration);
              if (cand_d2 < exist_d2)
              {
                target_sol.configurations[matched_acc_idx] = sol.configurations[c_idx];
                if (has_tra_err && matched_acc_idx < target_sol.translation_residual_errors.size())
                {
                  target_sol.translation_residual_errors[matched_acc_idx] = sol.translation_residual_errors[c_idx];
                }
                if (has_rot_err && matched_acc_idx < target_sol.rotation_residual_errors.size())
                {
                  target_sol.rotation_residual_errors[matched_acc_idx] = sol.rotation_residual_errors[c_idx];
                }
              }
            }
          }
          else
          {
            size_t new_idx = target_sol.configurations.size();
            target_sol.configurations.push_back(sol.configurations[c_idx]);
            if (has_tra_err)
            {
              target_sol.translation_residual_errors.push_back(sol.translation_residual_errors[c_idx]);
            }
            if (has_rot_err)
            {
              target_sol.rotation_residual_errors.push_back(sol.rotation_residual_errors[c_idx]);
            }

            buckets[b].push_back(new_idx);
          }
        }
      }
    }

    flat_req.targets.clear();
    flat_req.targets.shrink_to_fit();
    flat_res.solutions.clear();
    flat_res.solutions.shrink_to_fit();

    return true;
  }

  //========================================================================================
  std::vector<ik_solver::Solutions> IkServicesBase::computeIKArrayST(const std::vector<Eigen::Affine3d> &v_T_b_f,
                                                                     const std::vector<ik_solver::Configurations> &vseeds,
                                                                     size_t desired_solutions, int min_stall_iterations,
                                                                     int max_stall_iterations,
                                                                     uint8_t update_recursively_seeds)
  {
    std::vector<ik_solver::Solutions> ret(v_T_b_f.size());
    std::vector<ik_solver::Configurations> seeds = vseeds;
    const bool propagate_seeds =
        update_recursively_seeds != ik_solver_msgs::GetIkArray::Request::UPDATE_RECURSIVELY_SEEDS_DISABLE;

    size_t failed_poses_counter = 0;
    std::vector<size_t> numer_of_solutions;
    size_t ok_poses_counter = 0;
    for (size_t i = 0; i < v_T_b_f.size(); i++)
    {
      ik_solver::Solutions ik_sol = computeIkFunction(ik_solvers_.front().get(), v_T_b_f.at(i), seeds.at(i),
                                                      desired_solutions, min_stall_iterations, max_stall_iterations);

      if (ik_sol.configurations().size())
      {
        ret.at(i) = ik_sol;
        if (propagate_seeds && i < (v_T_b_f.size() - 1))
        {
          seeds.at(i + 1).insert(seeds.at(i + 1).begin(), ik_sol.configurations().begin(), ik_sol.configurations().end());
        }
        ok_poses_counter++;
      }
      else
      {
        failed_poses_counter++;
        ret.at(i).clear();
      }

      std::string nl = (i == v_T_b_f.size() - 1 ? "\n" : "");
      numer_of_solutions.push_back(ik_sol.configurations().size());
      size_t max_no_ik = *std::max_element(numer_of_solutions.begin(), numer_of_solutions.end());
      size_t min_no_ik = *std::min_element(numer_of_solutions.begin(), numer_of_solutions.end());

      std::string msg = ik_sol.message().empty() ? "" : ik_sol.message();
      msg += i == v_T_b_f.size() - 1 ? "\n" : "",

          ik_solver::printProgress(double(ok_poses_counter + failed_poses_counter) / double(v_T_b_f.size()),
                                   "IK ST - OK/KO/TOT %03zu/%03zu/%03zu (Min IK %02zu, Max IK %02zu) %s",
                                   ok_poses_counter, failed_poses_counter, v_T_b_f.size(), min_no_ik, max_no_ik, msg.c_str());
    }
    return ret;
  }
  //==========================================================promis=============================

  //========================================================================================
  std::vector<ik_solver::Solutions> IkServicesBase::computeIKArrayMT(const std::vector<Eigen::Affine3d> &v_T_b_f,
                                                                     const std::vector<ik_solver::Configurations> &vseeds,
                                                                     size_t desired_solutions, int min_stall_iterations,
                                                                     int max_stall_iterations, size_t num_threads)
  {
    const size_t n_poses = v_T_b_f.size();
    // Presized and never resized: each index is written by exactly one worker, so no lock is needed.
    std::vector<ik_solver::Solutions> ret(n_poses);
    if (n_poses == 0)
    {
      return ret;
    }

    std::atomic<size_t> next_index{0};
    std::atomic<size_t> ok_poses_counter{0};
    std::atomic<size_t> failed_poses_counter{0};
    std::atomic<size_t> done_counter{0};
    std::mutex done_mutex;
    std::condition_variable done_cv;

    // The pool has MAX_NUM_PARALLEL_IK_SOLVER solvers, one per thread: that is the upper limit.
    const size_t max_workers = (num_threads == 0 || num_threads > MAX_NUM_PARALLEL_IK_SOLVER)
                                   ? MAX_NUM_PARALLEL_IK_SOLVER
                                   : num_threads;
    const size_t n_workers = std::min(max_workers, n_poses);
    std::vector<size_t> min_no_ik(n_workers, std::numeric_limits<size_t>::max());
    std::vector<size_t> max_no_ik(n_workers, 0);

    auto thread_function = [&](size_t id)
    {
      ik_solver::IkSolver *solver = ik_solvers_.at(id).get();
      for (size_t i = next_index.fetch_add(1); i < n_poses; i = next_index.fetch_add(1))
      {
        size_t n_sol = 0;
        try
        {
          ret[i] = computeIkFunction(solver, v_T_b_f[i], vseeds[i], desired_solutions, min_stall_iterations,
                                     max_stall_iterations);
          n_sol = ret[i].configurations().size();
        }
        catch (const std::exception &e)
        {
          ret[i].clear();
          fprintf(stderr, "[ERROR] IK MT - pose %zu: solver threw: %s\n", i, e.what());
        }
        if (n_sol == 0)
        {
          ret[i].clear();
        }
        min_no_ik[id] = std::min(min_no_ik[id], n_sol);
        max_no_ik[id] = std::max(max_no_ik[id], n_sol);

        (n_sol ? ok_poses_counter : failed_poses_counter).fetch_add(1);
        if (done_counter.fetch_add(1) + 1 == n_poses)
        {
          std::lock_guard<std::mutex> lock(done_mutex);
          done_cv.notify_one();
        }
      }
    };

    std::vector<std::thread> workers;
    workers.reserve(n_workers);
    for (size_t thread_id = 0; thread_id < n_workers; thread_id++)
    {
      workers.emplace_back(thread_function, thread_id);
    }

    // Throttled progress: print at most every 100 ms instead of once per pose.
    {
      std::unique_lock<std::mutex> lock(done_mutex);
      while (!done_cv.wait_for(lock, std::chrono::milliseconds(100), [&]() { return done_counter.load() >= n_poses; }))
      {
        size_t ok = ok_poses_counter.load();
        size_t ko = failed_poses_counter.load();
        ik_solver::printProgress(double(ok + ko) / double(n_poses), "IK MT - OK/KO/TOT %03zu/%03zu/%03zu", ok, ko,
                                 n_poses);
      }
    }

    for (auto &w : workers)
    {
      if (w.joinable())
      {
        w.join();
      }
    }

    size_t min_ik = *std::min_element(min_no_ik.begin(), min_no_ik.end());
    size_t max_ik = *std::max_element(max_no_ik.begin(), max_no_ik.end());
    ik_solver::printProgress(1.0, "IK MT - OK/KO/TOT %03zu/%03zu/%03zu (Min IK %02zu, Max IK %02zu, threads %zu)\n",
                             ok_poses_counter.load(), failed_poses_counter.load(), n_poses, min_ik, max_ik, n_workers);

    return ret;
  }

  const ik_solver::IkSolver &IkServicesBase::config() const
  {
    return *ik_solvers_.front();
  }

  bool IkServicesBase::computeTransformations(const std::string &tip_frame, const std::string &reference_frame,
                                              Eigen::Affine3d &T_poses_base, Eigen::Affine3d &T_flange_tool,
                                              Eigen::Affine3d &T_tool_tip)
  {
    if (tip_frame.empty())
    {
      T_tool_tip.setIdentity();
    }
    else if (!config().getTF(config().tool_frame(), tip_frame, T_tool_tip))
    {
      //    ROS_ERROR("computeFKArray: error on computing TF from tool_name=%s, tip_frame=%s", config().tool_frame().c_str(),
      //              tip_frame.c_str());
      printf("%s[ERROR] computeFKArray: error on computing TF from tool_name=%s, tip_frame=%s %s", cnr_logger::RED().c_str(), config().tool_frame().c_str(),
             tip_frame.c_str(), cnr_logger::RESET().c_str());
      return false;
    }

    T_flange_tool = config().transform_from_flange_to_tool().inverse();
    if (!config().getTF(reference_frame, config().base_frame(), T_poses_base))
    {
      return false;
    }

    return true;
  }

  bool order_joint_names(const std::vector<std::string> &joint_names_ref, const std::vector<std::string> &joint_names_req,
                         std::vector<int> &order)
  {
    order.resize(joint_names_ref.size());
    for (int idx = 0; idx < joint_names_ref.size(); idx++)
    {
      bool found = false;
      for (int iax = 0; iax < joint_names_req.size(); iax++)
      {
        if (!joint_names_req.at(iax).compare(joint_names_ref.at(idx)))
        {
          found = true;
          // ROS_INFO("%s at  position %d",req->joint_names.at(iax).c_str(),iax);
          order.at(idx) = iax;
          break;
        }
      }
      if (!found)
      {
        //      ROS_ERROR("computeFKArray joint names are not correct");
        printf("%s[ERROR] computeFKArray joint names are not correct%s", cnr_logger::RED().c_str(), cnr_logger::RESET().c_str());
        return false;
      }
    }
    return true;
  }

  //========================================================================================
  bool IkServicesBase::computeFK(ik_solver_msgs::GetFk::Request *req, ik_solver_msgs::GetFk::Response *res)
  {
    Eigen::Affine3d T_poses_base;
    Eigen::Affine3d T_flange_tool;
    Eigen::Affine3d T_tool_tip;
    if (!computeTransformations(req->tip_frame, req->reference_frame, T_poses_base, T_flange_tool, T_tool_tip))
    {
      return false;
    }

    res->pose.header.frame_id = req->reference_frame;
    std::vector<int> order(config().joint_names().size());

    if (!order_joint_names(config().joint_names(), req->joint_names, order))
    {
      return false;
    }

    Configuration q(config().joint_names().size());
    for (int idx = 0; idx < config().joint_names().size(); idx++)
    {
      q(idx) = req->configuration.configuration.at(order.at(idx));
    }
    Eigen::Affine3d fk = T_poses_base * ik_solvers_.front()->getFK(q) * T_flange_tool * T_tool_tip;
    Pose p;
    from_eigen_to_pose(fk, p);
    res->pose.pose = p;

    return true;
  }

  //========================================================================================
  bool IkServicesBase::computeFKArray(ik_solver_msgs::GetFkArray::Request *req, ik_solver_msgs::GetFkArray::Response *res)
  {
    Eigen::Affine3d T_poses_base;
    Eigen::Affine3d T_flange_tool;
    Eigen::Affine3d T_tool_tip;
    if (!computeTransformations(req->tip_frame, req->reference_frame, T_poses_base, T_flange_tool, T_tool_tip))
    {
      return false;
    }

    res->poses.header.frame_id = req->reference_frame;
    std::vector<int> order(config().joint_names().size());

    if (!order_joint_names(config().joint_names(), req->joint_names, order))
    {
      return false;
    }

    const size_t n_poses = req->configurations.size();
    ik_solver::SafeQueue<std::size_t> fk_args_queue;
    std::vector<std::promise<Pose>> fk_sol_promises(n_poses);
    std::vector<std::future<Pose>> fk_sol_futures(n_poses);
    for (size_t i = 0; i < n_poses; i++)
    {
      fk_args_queue.enqueue(i);
      fk_sol_futures.at(i) = fk_sol_promises.at(i).get_future();
    }

    std::vector<std::thread> workers;
    for (size_t thread_id = 0; thread_id < MAX_NUM_PARALLEL_IK_SOLVER; thread_id++)
    {
      auto thread_function = [&](const size_t &id, const auto &order)
      {
        while (!fk_args_queue.empty())
        {
          size_t item_index;
          if (fk_args_queue.dequeue(item_index))
          {
            Configuration q(order.size());
            for (int idx = 0; idx < config().joint_names().size(); idx++)
            {
              q(idx) = req->configurations.at(item_index).configuration.at(order.at(idx));
            }
            auto solver = ik_solvers_.at(id).get();
            Eigen::Affine3d fk = T_poses_base * solver->getFK(q) * T_flange_tool * T_tool_tip;
            Pose p;
            from_eigen_to_pose(fk, p);
            fk_sol_promises.at(item_index).set_value(p);
          }
        }
      };
      workers.push_back(std::thread(thread_function, thread_id, order));
    }

    size_t failed_poses_counter = 0;
    while (res->poses.poses.size() + failed_poses_counter != n_poses)
    {
      std::chrono::milliseconds span(1000);
      for (auto it = fk_sol_futures.begin(); it != fk_sol_futures.end();)
      {
        if (it->wait_for(span) == std::future_status::ready)
        {
          auto fk_sol = it->get();
          res->poses.poses.push_back(fk_sol);
          it = fk_sol_futures.erase(it);

          ik_solver::printProgress(double(res->poses.poses.size() + failed_poses_counter) / double(n_poses),
                                   "FK MT - OK/FAILED/TOT %03zu/%03zu/%03zu", res->poses.poses.size(),
                                   failed_poses_counter, n_poses);
        }
        else
        {
          ++it;
        }
      }
      std::cout << std::endl;
    }

    try
    {
      for (size_t i = 0; i < workers.size(); i++)
      {
        if (workers.at(i).joinable())
        {
          workers.at(i).join();
        }
      }
    }
    catch (std::exception &e)
    {
      std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ":" << e.what() << std::endl;
    }
    return true;
  }

  /**
   * @brief
   *
   * @param req
   * @param res
   * @return true
   * @return false
   */

  bool IkServicesBase::getBounds(ik_solver_msgs::GetBound::Request *req, ik_solver_msgs::GetBound::Response *res)
  {
    res->boundaries.resize(config().ub().size());

    for (size_t iax = 0; iax < config().lb().size(); iax++)
    {
      res->boundaries.at(iax).joint_name = config().joint_names().at(iax);
      res->boundaries.at(iax).lower_bound = config().lb()(iax);
      res->boundaries.at(iax).upper_bound = config().ub()(iax);
    }
    return true;
  }

  bool IkServicesBase::setBounds(ik_solver_msgs::SetBound::Request *req, ik_solver_msgs::SetBound::Response *res)
  {
    const std::vector<std::string> &joint_names = config().joint_names();
    ik_solver::Configuration lb = config().lb();
    ik_solver::Configuration ub = config().ub();
    for (const auto &range : req->boundaries)
    {
      auto it = std::find(joint_names.begin(), joint_names.end(), range.joint_name);
      if (it == joint_names.end())
      {
        res->result = ik_solver_msgs::SetBound::Response::JOINT_NOT_FOUND;
        break;
      }
      const size_t iax = std::distance(joint_names.begin(), it);
      lb(iax) = range.lower_bound;
      ub(iax) = range.upper_bound;
    }

    if (res->result != ik_solver_msgs::SetBound::Response::JOINT_NOT_FOUND)
    {
      // Check on the first solver: all the solvers share the URDF, so they accept or reject the same limits.
      if (ik_solvers_.at(0)->setBounds(lb, ub))
      {
        std::for_each(ik_solvers_.begin() + 1, ik_solvers_.end(), [&lb, &ub](std::shared_ptr<IkSolver> &solver)
                      { solver->setBounds(lb, ub); });
        res->result = ik_solver_msgs::SetBound::Response::SUCCESS;
      }
      else
      {
        res->result = ik_solver_msgs::SetBound::Response::INVALID_BOUNDS;
      }
    }

    // Report the limits in use (changed or not)
    ik_solver_msgs::GetBound::Request get_req;
    ik_solver_msgs::GetBound::Response get_res;
    getBounds(&get_req, &get_res);
    res->boundaries = get_res.boundaries;
    return true;
  }

  /**
   * @brief
   *
   * @param req
   * @param res
   * @return true
   * @return false
   */

  bool IkServicesBase::getFrames(ik_solver_msgs::GetFrames::Request *req, ik_solver_msgs::GetFrames::Response *res)
  {
    res->base_frame = config().base_frame();
    res->flange_frame = config().flange_frame();
    res->tool_frame = config().tool_frame();

    return true;
  }
  /**
   * @brief
   *
   * @param req
   * @param res
   * @return true
   * @return false
   */
  bool IkServicesBase::reconfigure(Trigger::Request *req, Trigger::Response *res)
  {
    for (std::size_t i = 0; i < ik_solver::MAX_NUM_PARALLEL_IK_SOLVER; i++)
    {
      // Reload from the same namespace used at startup
      if (!ik_solvers_.at(i)->config(ik_solvers_.at(i)->param_namespace()))
      {
        //      ROS_ERROR("Unable to re-configure %s", nh_.getNamespace().c_str());
        printf("%s[ERROR] Unable to re-configure%s\n", cnr_logger::RED().c_str(), cnr_logger::RESET().c_str());
        res->success = false;
        res->message = "unable to re-configure solver " + std::to_string(i);
        return true;
      }
    }
    loadTaskRedundantParams();
    res->success = true;
    return true;
  }

  void IkServicesBase::changeTool(ik_solver_msgs::ChangeTool::Request *req, ik_solver_msgs::ChangeTool::Response *res)
  {
    Eigen::Affine3d T_tool_flange;
    if (!config().getTF(req->tool, config().flange_frame(), T_tool_flange))
    {
      res->result = ik_solver_msgs::ChangeTool::Response::TOOL_NOT_FOUND;
      return;
    }

    // Change tool for all solver in the pool
    std::for_each(ik_solvers_.begin(), ik_solvers_.end(), [&req, &T_tool_flange](std::shared_ptr<IkSolver> &solver)
                  { solver->changeTool(req->tool, T_tool_flange); });
    res->result = ik_solver_msgs::ChangeTool::Response::SUCCESS;
  }

  void IkServicesBase::loadTaskRedundantParams()
  {
    std::string what;
    std::string ns = config().param_namespace();
    if (!ns.empty() && ns.back() != '/')
    {
      ns += "/";
    }

    if (!cnr::param::get(ns + "filter_duplicates", filter_duplicates_, what))
    {
      if (!cnr::param::get(ns + "task_redundant/filter_duplicates", filter_duplicates_, what))
      {
        cnr::param::get(ns + "task_reduntant/filter_duplicates", filter_duplicates_, what);
      }
    }

    if (!initial_conf_set_manually_)
    {
      std::vector<double> conf;
      if (cnr::param::get(ns + "initial_conf", conf, what) ||
          cnr::param::get(ns + "initial_configuration", conf, what) ||
          cnr::param::get(ns + "task_redundant/initial_conf", conf, what) ||
          cnr::param::get(ns + "task_reduntant/initial_conf", conf, what))
      {
        initial_conf_ = conf;
      }
    }
  }

  bool IkServicesBase::setInitialConfiguration(ik_solver_msgs::SetInitialConfiguration::Request *req,
                                               ik_solver_msgs::SetInitialConfiguration::Response *res)
  {
    initial_conf_ = req->configuration.configuration;
    initial_conf_set_manually_ = true;
    res->success = true;
    res->message = "Initial configuration updated successfully.";
    return true;
  }

} // namespace ik_solver
