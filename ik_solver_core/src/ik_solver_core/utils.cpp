#include <cstddef>
#include <cstdarg>

#include <ik_solver_core/math.h>
#include <ik_solver_core/utils.h>


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

 bool isPresent(const Configuration& q, const Configurations& qq, double tolerance)
{
  for (const auto& _q : qq)
  {
    if (norm(diff(q, _q)) < std::fabs(tolerance))
      return true;
  }
  return false;
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







template <typename T>
bool _is_the_same(const T& lhs, const T& rhs)
{
  return rhs == lhs;
}

template <>
bool _is_the_same(const std::vector<std::string>& lhs, const std::vector<std::string>& rhs)
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

bool is_the_same(const std::vector<std::string>& lhs, const std::vector<std::string>& rhs)
{
  return _is_the_same(lhs,rhs);
}


bool is_the_same(const std::string& lhs, const std::string& rhs)
{
  return lhs == rhs;
}


}
