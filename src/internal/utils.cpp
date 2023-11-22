#include <ros/node_handle.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <ik_solver/internal/utils.h>
#include <cstddef>
#include <iterator>

namespace ik_solver
{

void printProgress(double percentage, const char* msg, ...)
{
  constexpr const size_t max_lenght = 256;
  char buffer[max_lenght] = { 0 };  // maximum expected length of the float
  va_list args;

  va_start(args, msg);
  // format the string
  vsnprintf(buffer, max_lenght, msg, args);
  va_end(args);

  int val = (int)(percentage * 100);
  int lpad = (int)(percentage * 60);
  int rpad = 60 - lpad;
  printf("\r%3d%% Ik Solver [%.*s%*s] %s", val, lpad, "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||", rpad, "", buffer);
  fflush(stdout);
}

// ===============
std::string ltrim(const std::string& s, const std::string& what)
{
  size_t start = s.find_first_not_of(what);
  return (start == std::string::npos) ? "" : s.substr(start);
}

std::string rtrim(const std::string& s, const std::string& what)
{
  size_t end = s.find_last_not_of(what);
  return (end == std::string::npos) ? "" : s.substr(0, end + 1);
}

std::string trim(const std::string& s, const std::string& what)
{
  return rtrim(ltrim(s));
}


 bool isPresent(const Configuration& q, const Configurations& qq, double tolerance)
{
  for (const auto& _q : qq)
  {
    if (norm(diff(q, _q)) < std::fabs(tolerance))
      return true;
  }
  return false;
}

std::string resolve_ns(const ros::NodeHandle& nh, const std::string& params_ns)
{
  std::string ret;
  bool is_private = params_ns.empty() ? false : params_ns[0] == '~';
  bool is_global = params_ns.empty() ? false : params_ns[0] == '/';
  bool is_relative = params_ns.empty() ? true : params_ns[0] == (!is_private && !is_global);

  if (is_private)
  {
    std::string _params_ns = params_ns;
    _params_ns[0] = '/';
    ret = ros::NodeHandle("~").getNamespace() + _params_ns;
  }
  else if (is_global)
  {
    ret = params_ns;
  }
  else
  {
    ret = nh.getNamespace() + "/" + params_ns;
  }

  ret = ret.back() == '/' ? ret : ret + "/";
  return ret;
}


bool getTF(const std::string& a_name, const std::string& b_name, Eigen::Affine3d& T_ab)
{
  tf::StampedTransform location_transform;
  ros::Time t0 = ros::Time(0);
  std::string tf_error;
  tf::TransformListener listener;
  if (!listener.waitForTransform(a_name, b_name, t0, ros::Duration(10), ros::Duration(0.01), &tf_error))
  {
    ROS_WARN("Unable to find a transform from %s to %s, tf error=%s", a_name.c_str(), b_name.c_str(), tf_error.c_str());
    return false;
  }

  try
  {
    listener.lookupTransform(a_name, b_name, t0, location_transform);
  }
  catch (...)
  {
    ROS_WARN("Unable to find a transform from %s to %s, tf error=%s", a_name.c_str(), b_name.c_str(), tf_error.c_str());
    return false;
  }

  tf::poseTFToEigen(location_transform, T_ab);
  return true;
}


Configurations getSeeds(const std::vector<std::string>& joint_names, const std::vector<std::string>& seed_names, const std::vector<ik_solver_msgs::Configuration>& seeds)
{
  Configurations seeds_eigen;
  Configurations solutions;

  bool seed_ok = true;
  if (seeds.size() > 0)
  {
    if (joint_names.size() != seed_names.size())
    {
      seed_ok = false;
    }
    else
    {
      for (size_t ij = 0; ij < joint_names.size(); ij++)
      {
        if (joint_names.at(ij).compare(seed_names.at(ij)))  // compare return true if different
        {
          seed_ok = false;
          break;
        }
      }
    }
  }

  if (!seed_ok)
  {
    ROS_WARN("seed names are wrong, skip seed");
  }
  else
  {
    for (const ik_solver_msgs::Configuration& seed : seeds)
    {
      Configuration s;
      s.resize(joint_names.size());
      if (s.size() != (long)seed.configuration.size())
      {
        ROS_WARN("seed dimensions are wrong, given %zu, expected %zu. Skip it", seed.configuration.size(), s.size());
        continue;
      }
      for (size_t idx = 0; idx < seed.configuration.size(); idx++)
      {
        s[idx] = seed.configuration.at(idx);
      }
      seeds_eigen.push_back(s);
    }
  }
  return seeds_eigen;
}

bool in_range(const double& v, const JointBoundaries& jb)
{
  if(std::isnan(v))
  {
    return false;
  }

  bool ok = true;
  for(const Range &  r : jb)
  {
    ok &= ((r.min() <= v) && (v <= r.max()));
  }
  return ok;
}

std::vector<std::pair<std::string, bool> > in_range(const Configuration& c, const JointsBoundaries& jb)
{
  std::vector<std::pair<std::string, bool> > ok(jb.size(), {"", false});
  for (size_t iax = 0; iax < jb.size(); iax++)
  {
    ok.at(iax).first = jb.at(iax).first;
     ok.at(iax).second = in_range(c(iax),jb.at(iax).second);
  }
  return ok;
}

bool all_in_range(const Configuration& sol, const JointsBoundaries& jb)
{
  auto vec = in_range(sol,jb);
  return std::all_of(vec.begin(), vec.end(), [](std::pair<std::string, bool> v) { return v.second; });
}

void outOfBound(const Configuration& c, const JointsBoundaries& jb, std::vector<int>& ax_out_of_bound)
{
  ax_out_of_bound.clear();
  auto vec = in_range(c,jb);
  for (size_t iax = 0; iax < jb.size(); iax++)
  {
    if (!vec.at(iax).second)
    {
      ax_out_of_bound.push_back(iax);
    }
  }
}

void outOfBound(const Configuration& c, const JointsBoundaries& jb, std::vector<std::string>& ax_out_of_bound)
{
  ax_out_of_bound.clear();
  auto vec = in_range(c,jb);
  for (size_t iax = 0; iax < jb.size(); iax++)
  {
    if (!vec.at(iax).second)
    {
      ax_out_of_bound.push_back(vec.at(iax).first);
    }
  }
}

void outOfBound(const Configuration& c, const JointsBoundaries& jb, std::vector<std::pair<std::string, int> >& ax_out_of_bound)
{
  ax_out_of_bound.clear();
  auto vec = in_range(c,jb);
  for (size_t iax = 0; iax < jb.size(); iax++)
  {
    if (!vec.at(iax).second)
    {
      ax_out_of_bound.push_back(std::make_pair(vec.at(iax).first, iax));
    }
  }
}

Configurations getMultiplicity(const Configurations& sol, const JointsBoundaries& jb, const std::vector<bool>& revolute)
{
  Configurations multiturn_global;

  for (const Configuration& q : sol)
  {
    Configurations multiturn;
    std::vector<std::vector<double>> multiturn_ax(jb.size());

    for (unsigned int idx = 0; idx < jb.size(); idx++)
    {
      multiturn_ax.at(idx).push_back(q(idx));

      if (!revolute.at(idx))
        continue;

      double tmp = q(idx);
      while (true)
      {
        tmp += 2 * M_PI;
        if (in_range(tmp, jb.at(idx).second))
        {
          multiturn_ax.at(idx).push_back(tmp);
        }
        else if(tmp> jb.at(idx).second.ub())
        {
          break;
        }
      }
      tmp = q(idx);
      while (true)
      {
        tmp -= 2 * M_PI;
        if (in_range(tmp, jb.at(idx).second))
        {
          multiturn_ax.at(idx).push_back(tmp);
        }
        else if(tmp <jb.at(idx).second.lb())
        {
          break;
        }
      }
    }

    if (!isPresent(q, multiturn))
    {
      multiturn.push_back(q);
    }

    for (unsigned int idx = 0; idx < jb.size(); idx++)
    {
      size_t size_multiturn = multiturn.size();
      for (size_t is = 1; is < multiturn_ax.at(idx).size(); is++)
      {
        for (size_t im = 0; im < size_multiturn; im++)
        {
          Eigen::VectorXd new_q = multiturn.at(im);
          new_q(idx) = multiturn_ax.at(idx).at(is);
          if (!isPresent(new_q, multiturn))
          {
            multiturn.push_back(new_q);
          }
        }
      }
    }

    for (const Eigen::VectorXd& q2 : multiturn)
    {
      if (!isPresent(q2, multiturn_global))
      {
        multiturn_global.push_back(q2);
      }
    }
  }

  return multiturn_global;
}

}