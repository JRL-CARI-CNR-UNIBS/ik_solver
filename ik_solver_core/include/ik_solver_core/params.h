#pragma once
/* Copyright (C) 2024 Beschi Manuel
 * SPDX-License-Identifier:    Apache-2.0
 */
/**
 * @file params.h
 * @brief Parameters passed to the IK solvers.
 *
 * ik_solver_core does not read parameters from any server. The application fills a
 * ParamMap (e.g. from the ROS 2 parameters of a node, see ik_solver/ros_params.hpp, or
 * from a YAML file with paramsFromYAML()) and passes it to IkSolverBase::config().
 * Keys are the parameter names relative to the solver, e.g. "base_frame"; nested names
 * use '.' as in ROS 2, e.g. "limits.shoulder_pan_joint.upper".
 */

#include <ik_solver_core/logger.h>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace ik_solver
{
/**
 * @brief Value of a parameter. Integers are stored as int64_t and arrays as
 * std::vector, as in ROS 2.
 */
using ParamValue = std::variant<bool, int64_t, double, std::string, std::vector<double>, std::vector<int64_t>,
                                std::vector<std::string>>;

/**
 * @brief Parameter name -> value.
 */
using ParamMap = std::map<std::string, ParamValue>;

/**
 * @brief Builds a ParamMap from a YAML map.
 *
 * Nested maps are flattened joining the keys with `separator`, e.g.
 * {a: {b: 1}} -> {"a.b": 1}. Scalars become int64_t, double, bool (only for
 * true/false) or std::string. Sequences of scalars become vectors (int64_t if
 * all elements are integers, double if all are numbers, std::string
 * otherwise). Other values (e.g. sequences of sequences, as in a saved
 * Path/Tree) cannot be stored and are skipped: load them with fromYAML().
 *
 * @param yaml a YAML map.
 * @param separator string used to join nested keys.
 * @return the parameters.
 * @throws std::invalid_argument if yaml is not a map.
 */
ParamMap paramsFromYAML(const YAML::Node& yaml, const std::string& separator = ".");

/**
 * @brief Returns the sub-map of the parameters starting with `prefix +
 * separator`, with the prefix removed from the keys.
 */
ParamMap subParams(const ParamMap& params, const std::string& prefix, const std::string& separator = ".");

namespace detail
{
// Own versions of std::type_identity_t and std::in_range, missing in GCC 9 (Ubuntu 20.04)
template <typename T>
struct type_identity
{
  using type = T;
};

template <typename T>
constexpr bool inRange(int64_t v)
{
  if constexpr (std::is_unsigned_v<T>)
    return v >= 0 && static_cast<uint64_t>(v) <= static_cast<uint64_t>(std::numeric_limits<T>::max());
  else
    return v >= static_cast<int64_t>(std::numeric_limits<T>::min()) &&
           v <= static_cast<int64_t>(std::numeric_limits<T>::max());
}

template <typename T>
struct is_std_vector : std::false_type
{
};
template <typename T, typename A>
struct is_std_vector<std::vector<T, A>> : std::true_type
{
};

template <typename T, typename = void>
struct is_eigen_vector : std::false_type
{
};
template <typename T>
struct is_eigen_vector<T, std::void_t<decltype(T::ColsAtCompileTime)>> : std::bool_constant<T::ColsAtCompileTime == 1>
{
};

inline const char* typeName(const ParamValue& value)
{
  static const char* names[] = {
    "bool", "integer", "double", "string", "double array", "integer array", "string array"
  };
  return names[value.index()];
}

template <typename T>
bool convertScalar(const ParamValue& value, T& out, std::string& what)
{
  if constexpr (std::is_same_v<T, bool>)
  {
    if (const bool* v = std::get_if<bool>(&value))
    {
      out = *v;
      return true;
    }
    what = "expected a bool";
    return false;
  }
  else if constexpr (std::is_integral_v<T>)
  {
    if (const int64_t* v = std::get_if<int64_t>(&value))
    {
      if (not inRange<T>(*v))
      {
        what = "value " + std::to_string(*v) + " is out of range for the requested integer type" +
               (std::is_unsigned_v<T> && *v < 0 ? " (negative value for an unsigned parameter)" : "");
        return false;
      }
      out = static_cast<T>(*v);
      return true;
    }
    what = "expected an integer";
    return false;
  }
  else if constexpr (std::is_floating_point_v<T>)
  {
    if (const double* v = std::get_if<double>(&value))
    {
      out = static_cast<T>(*v);
      return true;
    }
    if (const int64_t* v = std::get_if<int64_t>(&value))  // e.g. "1" written instead of "1.0"
    {
      out = static_cast<T>(*v);
      return true;
    }
    what = "expected a number";
    return false;
  }
  else if constexpr (std::is_same_v<T, std::string>)
  {
    if (const std::string* v = std::get_if<std::string>(&value))
    {
      out = *v;
      return true;
    }
    what = "expected a string";
    return false;
  }
  else
  {
    static_assert(sizeof(T) == 0, "ik_solver::get_param: unsupported parameter type");
    return false;
  }
}

/**
 * @brief Converts each element of `in` into an element of `out`.
 */
template <typename Out, typename In>
bool convertElements(const std::vector<In>& in, Out& out, std::string& what)
{
  for (size_t i = 0; i < in.size(); i++)
  {
    if (not convertScalar(ParamValue(in[i]), out[i], what))
    {
      what = "element " + std::to_string(i) + ": " + what;
      return false;
    }
  }
  return true;
}

template <typename T>
bool convert(const ParamValue& value, T& out, std::string& what)
{
  if constexpr (is_std_vector<T>::value || is_eigen_vector<T>::value)
  {
    // An array of any type is accepted; elements are converted one by one.
    // An empty array is read by ROS 2 and YAML without type information, so
    // any empty array is accepted.
    return std::visit(
        [&](const auto& v) -> bool {
          using V = std::decay_t<decltype(v)>;
          if constexpr (is_std_vector<V>::value)
          {
            if constexpr (is_eigen_vector<T>::value)
            {
              if (T::RowsAtCompileTime != Eigen::Dynamic && static_cast<Eigen::Index>(v.size()) != T::RowsAtCompileTime)
              {
                what = "expected an array with " + std::to_string(T::RowsAtCompileTime) + " elements, got " +
                       std::to_string(v.size());
                return false;
              }
              T tmp(static_cast<Eigen::Index>(v.size()));
              if (not convertElements(v, tmp, what))
                return false;
              out = tmp;
            }
            else
            {
              T tmp(v.size());
              if (not convertElements(v, tmp, what))
                return false;
              out = std::move(tmp);
            }
            return true;
          }
          else
          {
            what = std::string("expected an array, got a ") + typeName(value);
            return false;
          }
        },
        value);
  }
  else
  {
    if (convertScalar(value, out, what))
      return true;
    what += std::string(", got a ") + typeName(value);
    return false;
  }
}

template <typename T>
void print(std::ostream& os, const T& value)
{
  if constexpr (is_eigen_vector<T>::value)
    os << value.transpose();
  else if constexpr (is_std_vector<T>::value)
  {
    os << "[";
    for (size_t i = 0; i < value.size(); i++)
      os << (i == 0 ? "" : ", ") << value[i];
    os << "]";
  }
  else if constexpr (std::is_same_v<T, bool>)
    os << (value ? "true" : "false");
  else
    os << value;
}
}  // namespace detail

/**
 * @brief Reads a parameter from a ParamMap.
 *
 * Supported types: bool, integer types, floating point types, std::string,
 * std::vector of these, and Eigen vectors. Conversions: an integer is accepted
 * for a floating point parameter; integers are range-checked (e.g. a negative
 * value for an unsigned parameter is an error); an array of numbers is
 * accepted for an Eigen vector.
 *
 * @param logger logger used for warnings and errors (it can be null).
 * @param params the parameters.
 * @param name the parameter name.
 * @param[out] value the parameter value; unchanged if the parameter is missing.
 * @return true if the parameter exists, false (with a warning) otherwise.
 * @throws std::invalid_argument if the parameter exists but has a wrong type.
 */
template <typename T>
bool get_param(const LoggerPtr& logger, const ParamMap& params, const std::string& name, T& value)
{
  auto it = params.find(name);
  if (it == params.end())
  {
    IK_WARN(logger, name << " parameter not available.");
    return false;
  }

  std::string what;
  if (not detail::convert(it->second, value, what))
  {
    IK_ERROR(logger, "Cannot load " << name << " parameter: " << what);
    throw std::invalid_argument("Cannot load " + name + " parameter: " + what);
  }
  return true;
}

/**
 * @brief Same as get_param(logger, params, name, value), but if the parameter
 * is missing `value` is set to `default_value`.
 */
template <typename T>
bool get_param(const LoggerPtr& logger, const ParamMap& params, const std::string& name, T& value,
               const typename detail::type_identity<T>::type& default_value)
{
  auto it = params.find(name);
  if (it == params.end())
  {
    value = default_value;
    IK_WARN(
        logger, name << " parameter not available. Using " << name << " = " << [&] {
          std::ostringstream oss;
          detail::print(oss, default_value);
          return oss.str();
        }());
    return false;
  }
  return get_param(logger, params, name, value);
}

/**
 * @brief Reads an optional parameter: if it is missing, `value` is unchanged
 * and nothing is logged.
 * @return true if the parameter exists.
 * @throws std::invalid_argument if the parameter exists but has a wrong type.
 */
template <typename T>
bool get_optional_param(const LoggerPtr& logger, const ParamMap& params, const std::string& name, T& value)
{
  if (params.find(name) == params.end())
    return false;
  return get_param(logger, params, name, value);
}

}  // namespace ik_solver
