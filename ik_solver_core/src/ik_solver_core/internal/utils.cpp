#include <ik_solver_core/internal/utils.h>
#include <cstddef>
#include <stdarg.h>

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

std::string resolve_ns(const std::string& params_ns)
{
  std::string ret;

  bool is_private = params_ns.empty() ? false : params_ns[0] == '~';
  bool is_global = params_ns.empty() ? false : params_ns[0] == '/';
  bool is_relative = params_ns.empty() ? true : params_ns[0] == (!is_private && !is_global);

  // if (is_private)
  // {
  //   std::string _params_ns = params_ns;
  //   _params_ns[0] = '/';
  //   ret = ros::NodeHandle("~").getNamespace() + _params_ns;
  // }
  // else if (is_global)
  // {
  //   ret = params_ns;
  // }
  // else
  // {
  //   ret = nh.getNamespace() + "/" + params_ns;
  // }

  // Hack until cnr_param support private and relative parameters
  ret = params_ns;
  if(is_relative || is_private)
  {
    ret.front() == '/'? ret : "/" + ret;
  }

  ret = ret.back() == '/' ? ret : ret + "/";
  return ret;
}


Configurations getMultiplicity(const Configurations& sol, const Configuration& ub, const Configuration& lb, const std::vector<bool>& revolute)
{
  Configurations multiturn_global;

  for (const Configuration& q : sol)
  {
    Configurations multiturn;
    std::vector<std::vector<double>> multiturn_ax(ub.size());

    for (unsigned int idx = 0; idx < ub.size(); idx++)
    {
      multiturn_ax.at(idx).push_back(q(idx));

      if (!revolute.at(idx))
        continue;

      double tmp = q(idx);
      while (true)
      {
        tmp += 2 * M_PI;
        if (tmp > ub(idx))
          break;
        multiturn_ax.at(idx).push_back(tmp);
      }
      tmp = q(idx);
      while (true)
      {
        tmp -= 2 * M_PI;
        if (tmp < lb(idx))
          break;
        multiturn_ax.at(idx).push_back(tmp);
      }
    }

    if (!isPresent(q, multiturn))
      multiturn.push_back(q);

    for (unsigned int idx = 0; idx < ub.size(); idx++)
    {
      size_t size_multiturn = multiturn.size();
      for (size_t is = 1; is < multiturn_ax.at(idx).size(); is++)
      {
        for (size_t im = 0; im < size_multiturn; im++)
        {
          Eigen::VectorXd new_q = multiturn.at(im);
          new_q(idx) = multiturn_ax.at(idx).at(is);
          if (!isPresent(new_q, multiturn))
            multiturn.push_back(new_q);
        }
      }
    }

    for (const Eigen::VectorXd& q2 : multiturn)
      if (!isPresent(q2, multiturn_global))
        multiturn_global.push_back(q2);
  }

  return multiturn_global;
}

std::vector<int> outOfBound(const Configuration& c, const Configuration& ub, const Configuration& lb)
{
  std::vector<int> out_of_bound;
  for (int iax = 0; iax < lb.size(); iax++)
  {
    if ((c(iax) < lb(iax)) || (c(iax) > ub(iax)) || (std::isnan(c(iax))))
    {
      out_of_bound.push_back(iax);
    }
  }
  return out_of_bound;
}


}
