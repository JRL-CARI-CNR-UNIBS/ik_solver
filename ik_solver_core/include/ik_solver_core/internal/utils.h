/* Copyright (C) 2024 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */


#ifndef IK_SOLVER_CORE__INTERNAL__UITLS_H
#define IK_SOLVER_CORE__INTERNAL__UITLS_H

#include <numeric>
#include <string>
#include <vector>
#include <sstream>
#include <regex>
#include <cstdio>

#include <Eigen/Geometry>


#include <cnr_param/cnr_param.h>
#include <ik_solver_core/internal/types.h>

namespace std
{
// ===============
//inline std::string to_string(const std::string& s)
//{
//  return s;
//}

/**
 * @brief
 *
 * @param ss
 * @return std::string
 */
template<typename T>
inline std::string to_string(const std::vector<T>& ss)
{
  std::stringstream ret;
  ret << "[";
  for (const auto& s : ss)
    ret << s <<  ",";
  ret << "]";
  return ret.str();
}

inline std::string to_string(const Eigen::Affine3d& mat)
{
  std::stringstream ss;
  ss << mat.matrix();
  std::string s = ss.str();
  std::regex_replace(s, std::regex("\\r\\n|\\r|\\n"), ", ");
  return s;
}

inline std::string to_string(const ik_solver::Configuration& mat)
{
  std::stringstream ss;
  if (mat.rows() < mat.cols())
  {
    ss << mat.transpose();
  }
  return ss.str();
}

inline std::string to_string(const ik_solver::Configurations& seeds)
{
  std::string ret;
  for (const auto& s : seeds)
  {
    ret += to_string(s) + ", ";
  }
  return ret;
}
}  // namespace std

/**
 * @brief 
 * 
 */
namespace ik_solver
{

void printProgress(double percentage, const char* msg, ...);

std::string ltrim(const std::string& s, const std::string& what = " \n\r\t\f\v");

std::string rtrim(const std::string& s, const std::string& what = " \n\r\t\f\v");

std::string trim(const std::string& s, const std::string& what = " \n\r\t\f\v");

template <typename T>
inline T diff(const T& a, const T& b)
{
  assert(0);
}

template <typename T>
inline double norm(const T& vv)
{
  assert(0);
  return 0;
}

template <typename T>
bool is_the_same(const T& lhs, const T& rhs)
{
  return rhs == lhs;
}

template <typename T>
inline bool check_and_set_duplicate_params(const std::string& nh_ns, const std::string& robot_ns,
                                           const std::string& param_name, const T& nh_val)
{
  std::string pn = robot_ns + param_name;
  std::string param_what;
  T val;
  if (cnr::param::get(pn, val, param_what))
  {
    if (!is_the_same(val, nh_val))
    {
      printf("[ERROR]:"
          "%s has been found both in the Composed Robot namespace (%s) "
          "and in the Mounted Robot namespace (%s) but they are "
          "different (%s and %s). Put it only in the Composed Robot "
          "namespace, or alternatuively be sure they are the same!\n",
          pn.c_str(), nh_ns.c_str(), robot_ns.c_str(), to_string(val).c_str(), std::to_string(nh_val).c_str());
      return false;
    }
  }
  else
  {
    cnr::param::set(pn, nh_val, param_what);
  }
  return true;
}
//======================================================================================

template <typename T>
inline bool check_and_get(std::map<std::string, std::tuple<T*, bool, T>>& params)
{
  std::string param_what;
  for (auto& param : params)
  {
    T val = std::get<1>(param.second) ? std::get<2>(param.second) : T();
    if (!cnr::param::get(param.first, val, param_what))
    {
      printf("[ERROR]: %s\n", param_what.c_str());
      if (!std::get<1>(param.second))
      {
        printf("[ERROR]: [Check and Get] %s is not specified! Abort.\n", param.first.c_str());
        return false;
      }
      else
      {
        val = std::get<2>(param.second);
        printf("[DEBUG]: %s is not specified. Default value %s\n", param.first.c_str(), std::to_string(val).c_str());
      }
    }
    *std::get<0>(param.second) = val;
  }
  return true;
}

// NONE DEFAULT VALUE
template <typename T>
inline bool get_and_return(std::map<std::string, T*>& params)
{
  std::map<std::string, std::tuple<T*, bool, T>> _params;
  for (auto& param : params)
  {
    _params[param.first] = std::make_tuple(param.second, false, T());
  }
  return check_and_get(_params);
}

// ALL WITH DEFAULT VALUE
template <typename T>
inline bool get_and_default(std::map<std::string, std::pair<T*, T>>& params)
{
  std::map<std::string, std::tuple<T*, bool, T>> _params;
  for (auto& param : params)
  {
    _params[param.first] = std::make_tuple(param.second.first, true, param.second.second);
  }
  return check_and_get(_params);
}

bool isPresent(const Configuration& q, const Configurations& qq, double tolerance = 1e-4);

std::string resolve_ns(const std::string& params_ns);



template<typename Derived>
inline bool is_finite(const Eigen::MatrixBase<Derived>& x)
{
	return ( (x - x).array() == (x - x).array()).all();
}

template<typename Derived>
inline bool is_nan(const Eigen::MatrixBase<Derived>& x)
{
	return ((x.array() == x.array())).all();
}

// Configurations getSeeds(const std::vector<std::string>& joint_names, const std::vector<std::string>& seed_names, const std::vector<ik_solver_msgs::Configuration>& seeds);

Configurations getMultiplicity(const Configurations& sol, const Configuration& ub, const Configuration& lb, const std::vector<bool>& revolute);

std::vector<int> outOfBound(const Configuration& c, const Configuration& ub, const Configuration& lb);

}



//=======================
// TEMPLATE SPECIALIZATION
//=======================
namespace ik_solver 
{
template <>
inline std::vector<double> diff(const std::vector<double>& vv, const std::vector<double>& ww)
{
  assert(vv.size() == ww.size());

  std::vector<double> ret(vv.size(), 0.0);
  std::transform(std::begin(vv), std::end(vv), std::begin(ww), std::begin(ret),
                 [](double a, double b) { return a - b; });
  return ret;
}

template <>
inline Eigen::VectorXd diff(const Eigen::VectorXd& vv, const Eigen::VectorXd& ww)
{
  assert(vv.rows() == ww.rows());
  return vv - ww;
}




template <>
inline double norm(const std::vector<double>& vv)
{
  return std::inner_product(vv.begin(), vv.end(), vv.begin(), 0.0);
}

template <>
inline double norm(const Eigen::VectorXd& vv)
{
  return vv.norm();
}

template <typename T>
inline double& at(T& vv, size_t i)
{
  static double r; 
  assert(0);
  return r;
}

template <>
inline double& at(std::vector<double>& vv, size_t i)
{
  return vv.at(i);
}

template <>
inline double& at(Eigen::VectorXd& vv, size_t i)
{
  return vv(i);
}

template <>
inline bool is_the_same(const std::vector<std::string>& lhs, const std::vector<std::string>& rhs)
{
  if (lhs.size() != rhs.size())
  {
    return false;
  }

  for (size_t i = 0; i < lhs.size(); i++)
  {
    if (lhs.at(i) != rhs.at(i))
    {
      return false;
    }
  }
  return true;
}

}

#endif  // IK_SOLVER_CORE__INTERNAL__UITLS_H
