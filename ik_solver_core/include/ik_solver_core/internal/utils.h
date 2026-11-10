/* Copyright (C) 2024 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */


/**
 * @file utils.h
 * @brief Utilities of the IK solvers: string conversions and joint-space helpers.
 */
#ifndef IK_SOLVER_CORE__INTERNAL__UTILS_H
#define IK_SOLVER_CORE__INTERNAL__UTILS_H

#include <numeric>
#include <string>
#include <vector>
#include <sstream>
#include <regex>
#include <cstdio>

#include <Eigen/Geometry>

#include <ik_solver_core/internal/types.h>

namespace std
{
// ===============
// inline std::string to_string(const std::string& s)
// {
//   return s;
// }

/**
 * @brief
 *
 * @param ss
 * @return std::string
 */
// template<typename T>
// inline std::string to_string(const std::vector<T>& ss)
// {
//   std::stringstream ret;
//   ret << "[";
//   for (const auto& s : ss)
//     ret << s <<  ",";
//   ret << "]";
//   return ret.str();
// }

/**
 * @brief String representation of a transform matrix.
 */
inline std::string to_string(const Eigen::Affine3d& mat)
{
  std::stringstream ss;
  ss << mat.matrix();
  std::string s = ss.str();
  std::regex_replace(s, std::regex("\\r\\n|\\r|\\n"), ", ");
  return s;
}

/**
 * @brief String representation of a configuration (row vectors only; column vectors give an empty string).
 */
inline std::string to_string(const ik_solver::Configuration& mat)
{
  std::stringstream ss;
  if (mat.rows() < mat.cols())
  {
    ss << mat.transpose();
  }
  return ss.str();
}

/**
 * @brief Comma-separated representation of a list of configurations.
 */
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

/**
 * @brief Print a progress bar on stdout (same line).
 * @param percentage Progress in [0, 1].
 * @param msg printf-like message printed after the bar.
 */
void printProgress(double percentage, const char* msg, ...);

/**
 * @brief Remove the characters in @p what from the beginning of @p s.
 */
std::string ltrim(const std::string& s, const std::string& what = " \n\r\t\f\v");

/**
 * @brief Remove the characters in @p what from the end of @p s.
 */
std::string rtrim(const std::string& s, const std::string& what = " \n\r\t\f\v");

/**
 * @brief Remove whitespace from both ends of @p s (@p what is ignored).
 */
std::string trim(const std::string& s, const std::string& what = " \n\r\t\f\v");

template <typename T>
/**
 * @brief Difference a - b. Specialized for std::vector<double> and Eigen::VectorXd.
 */
inline T diff(const T& a, const T& b)
{
  assert(0);
}

template <typename T>
/**
 * @brief Norm of a vector. Specialized for std::vector<double> (squared norm) and Eigen::VectorXd (Euclidean norm).
 */
inline double norm(const T& vv)
{
  assert(0);
  return 0;
}

template <typename T>
/**
 * @brief Equality check, specialized for vectors of strings.
 */
bool is_the_same(const T& lhs, const T& rhs)
{
  return rhs == lhs;
}

/**
 * @brief Check if a configuration is in a list.
 * @param q Configuration.
 * @param qq List.
 * @param tolerance Distance below which two configurations are the same.
 * @return True if @p q is in @p qq.
 */
bool isPresent(const Configuration& q, const Configurations& qq, double tolerance = 1e-4);

/**
 * @brief Normalize a parameter namespace: add a leading '/' to relative or private names and a trailing '/'.
 */
std::string resolve_ns(const std::string& params_ns);



template<typename Derived>
/**
 * @brief True if all the coefficients are finite.
 */
inline bool is_finite(const Eigen::MatrixBase<Derived>& x)
{
	return ( (x - x).array() == (x - x).array()).all();
}

template<typename Derived>
/**
 * @brief True if no coefficient is NaN (note: despite the name).
 */
inline bool is_nan(const Eigen::MatrixBase<Derived>& x)
{
	return ((x.array() == x.array())).all();
}

// Configurations getSeeds(const std::vector<std::string>& joint_names, const std::vector<std::string>& seed_names, const std::vector<ik_solver_msgs::Configuration>& seeds);

/**
 * @brief Add the multi-turn equivalents of each solution.
 *
 * For each revolute joint, the values q ± 2kπ inside the joint limits are added, and all
 * combinations are returned (without duplicates).
 * @param sol Solutions.
 * @param ub Upper limits.
 * @param lb Lower limits.
 * @param revolute True for revolute joints.
 * @return Solutions and their multi-turn equivalents.
 */
Configurations getMultiplicity(const Configurations& sol, const Configuration& ub, const Configuration& lb, const std::vector<bool>& revolute);

/**
 * @brief Joints outside their limits (or NaN).
 * @return Indices of the joints out of bounds.
 */
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
/**
 * @brief Element access, specialized for std::vector<double> and Eigen::VectorXd.
 */
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

#endif  // IK_SOLVER_CORE__INTERNAL__UTILS_H
