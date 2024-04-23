/* Copyright (C) 2024 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <exception>
#include <future>
#include <iostream>
#include <iterator>
#include <optional>
#include <Eigen/Geometry>
#include <thread>
#include <vector>
#include "Eigen/src/Geometry/Transform.h"
#include "ik_solver/internal/SafeQueue.h"
#include "ik_solver_msgs/Configuration.h"
#include <geometry_msgs/Pose.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <ik_solver/ik_solver_base_class.h>
#include <ik_solver/internal/types.h>
#include <ik_solver/internal/utils.h>
#include <ik_solver/internal/services.h>

#include <ik_solver/internal/SafeQueue.h>

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
ik_solver::Solutions computeIkFunction(ik_solver::IkSolver* solver, const Eigen::Affine3d& T_b_f,
                                       const ik_solver::Configurations& seeds, const int& desired_solutions,
                                       const int& min_stall_iterations, const int& max_stall_iterations)
{
  ik_solver::Solutions solutions;
  solutions = solver->getIk(T_b_f, seeds, desired_solutions, min_stall_iterations, max_stall_iterations);
  return solutions;
}

//========================================================================================

/**
 * @brief Construct a new Ik Services:: Ik Services object
 *
 * @param nh
 * @param ik_solvers
 */
IkServices::IkServices(ros::NodeHandle& nh, IkSolversPool& ik_solvers) : nh_(nh), ik_solvers_(ik_solvers)
{
  ik_server_ = nh.advertiseService("get_ik", &IkServices::computeIK, this);
  ik_server_array_ = nh.advertiseService("get_ik_array", &IkServices::computeIKArray, this);
  fk_server_ = nh.advertiseService("get_fk", &IkServices::computeFK, this);
  fk_server_array_ = nh.advertiseService("get_fk_array", &IkServices::computeFKArray, this);
  bound_server_array_ = nh.advertiseService("get_bounds", &IkServices::getBounds, this);
  reconfigure_ = nh.advertiseService("reconfigure", &IkServices::reconfigure, this);
}

/**
 * @brief
 *
 * @param req
 * @param res
 * @return true
 * @return false
 */
bool IkServices::computeIK(ik_solver_msgs::GetIk::Request& req, ik_solver_msgs::GetIk::Response& res)
{
  // The input pose is the TOOL pose
  // Frame {b}: robot base
  // Frame {r}: the frame id of the requested tool poses. It must be a solvable TF transform. No relative Transform.
  // TODO add check! Frame {t}: tool frame Frame {f}: flange frame
  Eigen::Affine3d T_b_r;
  if (!getTF(config().base_frame(), req.target.pose.header.frame_id, T_b_r))
  {
    res.success = false;
    res.message = "Failed in getting the TF from '"+config().base_frame()+"' to '"+req.target.pose.header.frame_id+"'";
    printf(  "Error: %s", res.message.c_str());
    return true;
  }
  Eigen::Affine3d T_r_t;
  tf::poseMsgToEigen(req.target.pose.pose, T_r_t);

  Eigen::Affine3d T_t_f = config().transform_from_flange_to_tool();
  Eigen::Affine3d T_b_f = T_b_r * T_r_t * T_t_f;

  Configurations seeds = ik_solver::getSeeds(config().joint_names(), req.seed_joint_names, req.target.seeds);

  int desired_solutions =
      (req.max_number_of_solutions > 0) ? req.max_number_of_solutions : config().desired_solutions();
  int min_stall_iterations = (req.stall_iterations > 0) ? req.stall_iterations : config().min_stall_iterations();
  int max_stall_iterations = (req.stall_iterations > 0) ? req.stall_iterations : config().max_stall_iterations();

  Solutions solutions = computeIkFunction(ik_solvers_.front().get(), T_b_f, seeds, desired_solutions,
                                          min_stall_iterations, max_stall_iterations);

  std::string msg = solutions.message().empty() ? "" : solutions.message();

  printf(  "IK - Number of solutions %02zu, %s\n", solutions.configurations().size(), msg.c_str() );
  fflush(stdout);
  
  res.solution = ik_solver::cast(solutions);
  res.joint_names = config().joint_names();
  res.success = true;
  res.message = "OK";
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
bool IkServices::computeIKArray(ik_solver_msgs::GetIkArray::Request& req, ik_solver_msgs::GetIkArray::Response& res)
{  // NOTE:
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
  for (const auto& t : req.targets)
  {
    Eigen::Affine3d T_b_r;
    if (T_b_rp.count(t.pose.header.frame_id) == 0)
    {
      if (!getTF(config().base_frame(), t.pose.header.frame_id, T_b_r))
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
    tf::poseMsgToEigen(t.pose.pose, T_r_t);

    Eigen::Affine3d T_t_f = config().transform_from_flange_to_tool();
    Eigen::Affine3d T_b_f = T_b_r * T_r_t * T_t_f;
    vseeds.push_back(ik_solver::getSeeds(config().joint_names(), req.seed_joint_names, t.seeds));
    v_T_b_f.push_back(T_b_f);
  }

  //===============================
  uint8_t parallelize = req.parallelize != ik_solver_msgs::GetIkArray::Request::PARALLELIZE_DEFAULT ?
                            req.parallelize :
                            config().parallelize();

  int update_recursively_seeds = ik_solver_msgs::GetIkArray::Request::UPDATE_RECURSIVELY_SEEDS_FORCE;
  ros::param::get(config().param_namespace() + "/update_recursively_seeds", update_recursively_seeds);
  update_recursively_seeds =
      req.update_recursively_seeds != ik_solver_msgs::GetIkArray::Request::UPDATE_RECURSIVELY_SEEDS_DEFAULT ?
          req.update_recursively_seeds :
          update_recursively_seeds;

  if (parallelize != ik_solver_msgs::GetIkArray::Request::PARALLELIZE_FORCE)
  {
    res.solutions =
        ik_solver::cast(computeIKArrayST(v_T_b_f, vseeds, config().desired_solutions(), config().min_stall_iterations(),
                                         config().max_stall_iterations(), req.update_recursively_seeds));
  }
  else
  {
    res.solutions = ik_solver::cast(computeIKArrayMT(v_T_b_f, vseeds, config().desired_solutions(),
                                                     config().min_stall_iterations(), config().max_stall_iterations()));
  }
  res.joint_names = config().joint_names();

  return true;
}

//========================================================================================
std::vector<ik_solver::Solutions> IkServices::computeIKArrayST(const std::vector<Eigen::Affine3d>& v_T_b_f,
                                                               const std::vector<ik_solver::Configurations>& vseeds,
                                                               size_t desired_solutions, int min_stall_iterations,
                                                               int max_stall_iterations,
                                                               uint8_t update_recursively_seeds)
{
  std::vector<ik_solver::Solutions> ret(v_T_b_f.size());
  std::vector<ik_solver::Configurations> seeds = vseeds;

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
      if (i < (v_T_b_f.size() - 1))
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
    msg += i == v_T_b_f.size()-1 ? "\n" : "",

    ik_solver::printProgress(double(ok_poses_counter + failed_poses_counter) / double(v_T_b_f.size()),
                              "IK ST - OK/KO/TOT %03zu/%03zu/%03zu (Min IK %02zu, Max IK %02zu) %s",
                              ok_poses_counter, failed_poses_counter, v_T_b_f.size(), min_no_ik, max_no_ik, msg.c_str());
  }
  return ret;
}
//==========================================================promis=============================

//========================================================================================
std::vector<ik_solver::Solutions> IkServices::computeIKArrayMT(const std::vector<Eigen::Affine3d>& v_T_b_f,
                                                               const std::vector<ik_solver::Configurations>& vseeds,
                                                               size_t desired_solutions, int min_stall_iterations,
                                                               int max_stall_iterations)
{
  assert(v_T_b_f.size());
  std::vector<ik_solver::Solutions> ret(v_T_b_f.size());
  ik_solver::SafeQueue<std::size_t> ik_args_queue;
  std::vector<IkArgs> ik_args(v_T_b_f.size());
  std::vector<std::promise<ik_solver::Solutions>> ik_sol_promises(v_T_b_f.size());
  std::vector<std::future<ik_solver::Solutions>> ik_sol_futures(v_T_b_f.size());
  std::vector<bool> ik_sol_futures_got(v_T_b_f.size(), false);

  for (size_t i = 0; i < v_T_b_f.size(); i++)
  {
    ik_args_queue.enqueue(i);
    ik_args.at(i) =
        std::make_tuple(v_T_b_f.at(i), vseeds.at(i), desired_solutions, min_stall_iterations, max_stall_iterations);
    ik_sol_futures.at(i) = ik_sol_promises.at(i).get_future();
  }

  std::vector<std::thread> workers;
  for (size_t thread_id = 0; thread_id < MAX_NUM_PARALLEL_IK_SOLVER; thread_id++)
  {
    auto thread_function = [&](const size_t& id) {
      while (!ik_args_queue.empty())
      {
        size_t item_index;
        if (ik_args_queue.dequeue(item_index))
        {
          IkArgs args = ik_args.at(item_index);
          auto solver = ik_solvers_.at(id).get();
          Solutions sols =
              computeIkFunction(solver, get_T_base_flange(args), get_seeds(args), get_desired_solutions(args),
                                get_min_stall_iterations(args), get_max_stall_iterations(args));
          ik_sol_promises.at(item_index).set_value(sols);
        }
      }
    };
    workers.push_back(std::thread(thread_function, thread_id));
  }

  size_t failed_poses_counter = 0;
  size_t ok_poses_counter = 0;
  std::vector<size_t> numer_of_solutions;
  while (ok_poses_counter + failed_poses_counter != v_T_b_f.size())
  {
    std::chrono::milliseconds span(1000);
    for (auto it = ik_sol_futures.begin(); it != ik_sol_futures.end();)
    {
      std::size_t item_index = std::distance(ik_sol_futures.begin(), it);
      if (!ik_sol_futures_got.at(item_index) && it->wait_for(span) == std::future_status::ready)
      {
        auto ik_sol = it->get();
        if (std::get<0>(ik_sol).size())
        {
          ok_poses_counter++;
          ret.at(item_index) = ik_sol;
        }
        else
        {
          failed_poses_counter++;
        }
        ik_sol_futures_got.at(item_index) = true;
        // it = ik_sol_futures.erase(it);


        numer_of_solutions.push_back(ik_sol.configurations().size());
        size_t max_no_ik = *std::max_element(numer_of_solutions.begin(), numer_of_solutions.end());
        size_t min_no_ik = *std::min_element(numer_of_solutions.begin(), numer_of_solutions.end());


        std::string msg = ik_sol.message().empty() ? "" : ik_sol.message();
        msg += ok_poses_counter + failed_poses_counter == v_T_b_f.size() ? "\n" : "",

        ik_solver::printProgress(double(ok_poses_counter + failed_poses_counter) / double(v_T_b_f.size()),
                              "IK MT - OK/KO/TOT %03zu/%03zu/%03zu (Min IK %02zu, Max IK %02zu) %s",
                              ok_poses_counter, failed_poses_counter, v_T_b_f.size(), min_no_ik, max_no_ik, msg.c_str());
      }
      else
      {
        ++it;
      }
    }
  }

  try
  {
    for(size_t i=0;i<workers.size();i++)
    {
      if(workers.at(i).joinable())
      {
        workers.at(i).join();
      }
    }
  }
  catch(std::exception& e)
  {
    std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ":" << e.what() << std::endl;
  }

  return ret;
}

const ik_solver::IkSolver& IkServices::config() const
{
  return *ik_solvers_.front();
}

bool IkServices::computeTransformations(const std::string& tip_frame, const std::string& reference_frame,
                                        Eigen::Affine3d& T_poses_base, Eigen::Affine3d& T_flange_tool,
                                        Eigen::Affine3d& T_tool_tip)
{
  if (tip_frame.empty())
  {
    T_tool_tip.setIdentity();
  }
  else if (!getTF(config().tool_frame(), tip_frame, T_tool_tip))
  {
    ROS_ERROR("computeFKArray: error on computing TF from tool_name=%s, tip_frame=%s", config().tool_frame().c_str(),
              tip_frame.c_str());
    return false;
  }

  T_flange_tool = config().transform_from_flange_to_tool().inverse();
  if (!getTF(reference_frame, config().base_frame(), T_poses_base))
  {
    return false;
  }

  return true;
}

bool order_joint_names(const std::vector<std::string>& joint_names_ref, const std::vector<std::string>& joint_names_req,
                       std::vector<int>& order)
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
        // ROS_INFO("%s at  position %d",req.joint_names.at(iax).c_str(),iax);
        order.at(idx) = iax;
        break;
      }
    }
    if (!found)
    {
      ROS_ERROR("computeFKArray joint names are not correct");
      return false;
    }
  }
  return true;
}

//========================================================================================
bool IkServices::computeFK(ik_solver_msgs::GetFk::Request& req, ik_solver_msgs::GetFk::Response& res)
{
  Eigen::Affine3d T_poses_base;
  Eigen::Affine3d T_flange_tool;
  Eigen::Affine3d T_tool_tip;
  if (!computeTransformations(req.tip_frame, req.reference_frame, T_poses_base, T_flange_tool, T_tool_tip))
  {
    return false;
  }

  res.pose.header.frame_id = req.reference_frame;
  std::vector<int> order(config().joint_names().size());

  if (!order_joint_names(config().joint_names(), req.joint_names, order))
  {
    return false;
  }

  Configuration q(config().joint_names().size());
  for (int idx = 0; idx < config().joint_names().size(); idx++)
  {
    q(idx) = req.configuration.configuration.at(order.at(idx));
  }
  Eigen::Affine3d fk = T_poses_base * ik_solvers_.front()->getFK(q) * T_flange_tool * T_tool_tip;
  geometry_msgs::Pose p;
  tf::poseEigenToMsg(fk, p);
  res.pose.pose = p;

  return true;
}

//========================================================================================
bool IkServices::computeFKArray(ik_solver_msgs::GetFkArray::Request& req, ik_solver_msgs::GetFkArray::Response& res)
{
  Eigen::Affine3d T_poses_base;
  Eigen::Affine3d T_flange_tool;
  Eigen::Affine3d T_tool_tip;
  if (!computeTransformations(req.tip_frame, req.reference_frame, T_poses_base, T_flange_tool, T_tool_tip))
  {
    return false;
  }

  res.poses.header.frame_id = req.reference_frame;
  std::vector<int> order(config().joint_names().size());

  if (!order_joint_names(config().joint_names(), req.joint_names, order))
  {
    return false;
  }

  const size_t n_poses = req.configurations.size();
  ik_solver::SafeQueue<std::size_t> fk_args_queue;
  std::vector<std::promise<geometry_msgs::Pose>> fk_sol_promises(n_poses);
  std::vector<std::future<geometry_msgs::Pose>> fk_sol_futures(n_poses);
  for (size_t i = 0; i < n_poses; i++)
  {
    fk_args_queue.enqueue(i);
    fk_sol_futures.at(i) = fk_sol_promises.at(i).get_future();
  }

  std::vector<std::thread> workers;
  for (size_t thread_id = 0; thread_id < MAX_NUM_PARALLEL_IK_SOLVER; thread_id++)
  {
    auto thread_function = [&](const size_t& id, const auto& order) {
      while (!fk_args_queue.empty())
      {
        size_t item_index;
        if (fk_args_queue.dequeue(item_index))
        {
          Configuration q(order.size());
          for (int idx = 0; idx < config().joint_names().size(); idx++)
          {
            q(idx) = req.configurations.at(item_index).configuration.at(order.at(idx));
          }
          auto solver = ik_solvers_.at(id).get();
          Eigen::Affine3d fk = T_poses_base * solver->getFK(q) * T_flange_tool * T_tool_tip;
          geometry_msgs::Pose p;
          tf::poseEigenToMsg(fk, p);
          fk_sol_promises.at(item_index).set_value(p);
        }
      }
    };
    workers.push_back(std::thread(thread_function, thread_id, order));
  }

  size_t failed_poses_counter = 0;
  while (res.poses.poses.size() + failed_poses_counter != n_poses)
  {
    std::chrono::milliseconds span(1000);
    for (auto it = fk_sol_futures.begin(); it != fk_sol_futures.end();)
    {
      if (it->wait_for(span) == std::future_status::ready)
      {
        auto fk_sol = it->get();
        res.poses.poses.push_back(fk_sol);
        it = fk_sol_futures.erase(it);

        ik_solver::printProgress(double(res.poses.poses.size() + failed_poses_counter) / double(n_poses),
                                 "FK MT - OK/FAILED/TOT %03zu/%03zu/%03zu", res.poses.poses.size(),
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
    for(size_t i=0;i<workers.size();i++)
    {
      if(workers.at(i).joinable())
      {
        workers.at(i).join();
      }
    }
  }
  catch(std::exception& e)
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

bool IkServices::getBounds(ik_solver_msgs::GetBound::Request& req, ik_solver_msgs::GetBound::Response& res)
{
  res.joint_names.resize(config().joint_names().size());
  res.lower_bound.resize(config().lb().size());
  res.upper_bound.resize(config().ub().size());

  for (size_t iax = 0; iax < config().lb().size(); iax++)
  {
    res.joint_names.at(iax) = config().joint_names().at(iax);
    res.lower_bound.at(iax) = config().lb()(iax);
    res.upper_bound.at(iax) = config().ub()(iax);
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
bool IkServices::reconfigure(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  for (std::size_t i = 0; i < ik_solver::MAX_NUM_PARALLEL_IK_SOLVER; i++)
  {
    if (!ik_solvers_.at(i)->config(nh_))
    {
      ROS_ERROR("Unable to re-configure %s", nh_.getNamespace().c_str());
      return 0;
    }
  }
  return true;
}

}  // namespace ik_solver
