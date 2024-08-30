#ifndef IK_SOLVER_CORE__INTERNAL__TYPES_H
#define IK_SOLVER_CORE__INTERNAL__TYPES_H

#include <cstddef>
#include <vector>
#include <Eigen/Core>

namespace ik_solver{
  using Configuration = Eigen::VectorXd;
  using Configurations = std::vector<Configuration>;
  struct Solutions : std::tuple<ik_solver::Configurations, std::vector<double>, std::vector<double>, std::string>
  {
    const ik_solver::Configurations& configurations() const {return std::get<0>(*this);}
    ik_solver::Configurations& configurations() {return std::get<0>(*this);}
  
    const std::vector<double>& translation_residuals() const {return std::get<1>(*this);}
    std::vector<double>& translation_residuals() {return std::get<1>(*this);}
  
    const std::vector<double>& rotation_residuals() const {return std::get<2>(*this);}
    std::vector<double>& rotation_residuals() {return std::get<2>(*this);}
  
    const std::string& message() const { return std::get<3>(*this); }
    std::string& message() { return std::get<3>(*this); }
  
    void clear() { configurations().clear(); translation_residuals().clear(); rotation_residuals().clear(); message().clear();}
  
  };
}

#endif //IK_SOLVER_CORE__INTERNAL__TYPES_H