#include <ik_solver_core/utils.h>

#include <regex>

namespace std
{

std::string to_string(const std::string& s)
{
  return s;
}

/**
 * @brief
 *
 * @param ss
 * @return std::string
 */
template<typename T>
std::string to_string(const std::vector<T>& ss)
{
  std::stringstream ret;
  ret << "[";
  for (const auto& s : ss)
    ret << s <<  ",";
  ret << "]";
  return ret.str();
}

std::string to_string(const Eigen::Affine3d& mat)
{
  std::stringstream ss;
  ss << mat.matrix();
  std::string s = ss.str();
  std::regex_replace(s, std::regex("\\r\\n|\\r|\\n"), ", ");
  return s;
}

std::string to_string(const geometry_msgs::Pose& mat)
{
  std::stringstream ss;
  ss << mat;
  std::string s = ss.str();
  std::regex_replace(s, std::regex("\\r\\n|\\r|\\n"), ", ");
  return s;
}

std::string to_string(const ik_solver::Configuration& mat)
{
  std::stringstream ss;
  if (mat.rows() < mat.cols())
  {
    ss << mat.transpose();
  }
  return ss.str();
}

std::string to_string(const ik_solver::Configurations& seeds)
{
  std::string ret;
  for (const auto& s : seeds)
  {
    ret += to_string(s) + ", ";
  }
  return ret;
}


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
  return rtrim(ltrim(s,what),what);
}

}  // namespace std
