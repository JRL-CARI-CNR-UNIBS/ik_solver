#include <algorithm>
#include <numeric>
#include <ik_solver_core/math.h>

namespace ik_solver 
{

//=======================
// TEMPLATE SPECIALIZATION
//=======================

template <typename T>
T _diff(const T& a, const T& b)
{
  assert(0);
}


template <>
std::vector<double> _diff(const std::vector<double>& vv, const std::vector<double>& ww)
{
  assert(vv.size() == ww.size());

  std::vector<double> ret(vv.size(), 0.0);
  std::transform(std::begin(vv), std::end(vv), std::begin(ww), std::begin(ret),
                 [](double a, double b) { return a - b; });
  return ret;
}

template <>
Eigen::VectorXd _diff(const Eigen::VectorXd& vv, const Eigen::VectorXd& ww)
{
  assert(vv.rows() == ww.rows());
  return vv - ww;
}


std::vector<double> diff(const std::vector<double>& vv, const std::vector<double>& ww)
{
  return _diff(vv,ww);
}

Eigen::VectorXd diff(const Eigen::VectorXd& vv, const Eigen::VectorXd& ww)
{
  return _diff(vv,ww);
}


template <typename T>
double _norm(const T& vv)
{
  assert(0);
  return 0;
}

template <>
double _norm(const std::vector<double>& vv)
{
  return std::inner_product(vv.begin(), vv.end(), vv.begin(), 0.0);
}

template <>
double _norm(const Eigen::VectorXd& vv)
{
  return vv.norm();
}

double norm(const std::vector<double>& vv)
{
  return _norm(vv);
}

double norm(const Eigen::VectorXd& vv)
{
  return _norm(vv);
}

template <typename T>
double& _at(T& vv, size_t i)
{
  static double r; 
  assert(0);
  return r;
}

template <>
double& _at(std::vector<double>& vv, size_t i)
{
  return vv.at(i);
}

template <>
double& _at(Eigen::VectorXd& vv, size_t i)
{
  return vv(i);
}

double& at(std::vector<double>& vv, size_t i)
{
  return _at(vv,i);
}

double& at(Eigen::VectorXd& vv, size_t i)
{
  return _at(vv,i);
}

}
