/*
Copyright (c) 2022, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef IK_SOLVER__IK_SOLVER_ROS__INTERNAL__UITLS_H
#define IK_SOLVER__IK_SOLVER_ROS__INTERNAL__UITLS_H

#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <ros/param.h>

#include <geometry_msgs/Pose.h>
#include <ik_solver_msgs/Configuration.h>
#include <ik_solver_msgs/IkSolution.h>
#include <ik_solver_core/types.h>
#include <ik_solver_core/string.h>
#include <ik_solver_core/ik_solver_base_class.h>

/**
 * @brief
 *
 */
namespace ik_solver
{
template <typename T>
bool check_and_set_duplicate_params(const std::string& nh_ns, const std::string& robot_ns,
                                    const std::string& param_name, const T& nh_val);

template <typename T>
bool check_and_get(std::map<std::string, std::tuple<T*, bool, T>>& params);

template <typename T>
bool get_and_return(std::map<std::string, T*>& params);

template <typename T>
bool get_and_default(std::map<std::string, std::pair<T*, T>>& params);

std::string resolve_ns(const ros::NodeHandle& nh, const std::string& params_ns);

bool getTF(const std::string& a_name, const std::string& b_name, Eigen::Affine3d& T_ab);

Configurations getSeeds(const std::vector<std::string>& joint_names, const std::vector<std::string>& seed_names,
                        const std::vector<ik_solver_msgs::Configuration>& seeds);

}  // namespace ik_solver

//=======================
// TEMPLATE SPECIALIZATION
//=======================
namespace ik_solver
{
template <typename T>
inline bool check_and_set_duplicate_params(const std::string& nh_ns, const std::string& robot_ns,
                                           const std::string& param_name, const T& nh_val, std::string& what)
{
  std::string pn = robot_ns + param_name;
  T val;
  if (ros::param::get(pn, val))
  {
    if (!is_the_same(val, nh_val))
    {
      what = pn + "has been found both in the Composed Robot namespace (" + nh_ns +
             ") and in the Mounted Robot namespace (" + robot_ns + ") but they are different (" + std::to_string(val) +
             " and " + std::to_string(nh_val) +
             "). Put it only in the Composed Robot namespace, or alternatuively be sure they are the same!";
      return false;
    }
  }
  else
  {
    ros::param::set(pn, nh_val);
  }
  return true;
}
//======================================================================================

template <typename T>
inline bool check_and_get(std::map<std::string, std::tuple<T*, bool, T>>& params, std::string& what)
{
  for (auto& param : params)
  {
    T val = std::get<1>(param.second) ? std::get<2>(param.second) : T();
    if (!ros::param::get(param.first, val))
    {
      if (!std::get<1>(param.second))
      {
        what = "The param " + param.first + "is not specified! Abort.";
        return false;
      }
      else
      {
        val = std::get<2>(param.second);
        what += std::string(what.length() > 0 ? "\n" : "") + "The param " + param.first +
                "is not specified. Default value is " + std::to_string(val);
      }
    }
    *std::get<0>(param.second) = val;
  }
  return true;
}

// NONE DEFAULT VALUE
template <typename T>
inline bool get_and_return(std::map<std::string, T*>& params, std::string& what)
{
  std::map<std::string, std::tuple<T*, bool, T>> _params;
  for (auto& param : params)
  {
    _params[param.first] = std::make_tuple(param.second, false, T());
  }
  return check_and_get(_params, what);
}

// ALL WITH DEFAULT VALUE
template <typename T>
inline bool get_and_default(std::map<std::string, std::pair<T*, T>>& params, std::string& what)
{
  std::map<std::string, std::tuple<T*, bool, T>> _params;
  for (auto& param : params)
  {
    _params[param.first] = std::make_tuple(param.second.first, true, param.second.second);
  }
  return check_and_get(_params, what);
}

}  // namespace ik_solver

#endif  // IK_SOLVER__IK_SOLVER_ROS__INTERNAL__UITLS_H
