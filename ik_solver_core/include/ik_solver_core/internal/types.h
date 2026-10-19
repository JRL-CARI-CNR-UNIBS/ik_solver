/**
 * @file types.h
 * @brief Basic types of the IK solvers.
 */
#ifndef IK_SOLVER_CORE__INTERNAL__TYPES_H
#define IK_SOLVER_CORE__INTERNAL__TYPES_H

#include <cstddef>
#include <vector>
#include <Eigen/Core>

namespace ik_solver{
  /**
   * @brief A joint configuration.
   */
  using Configuration = Eigen::VectorXd;
  /**
   * @brief A list of joint configurations.
   */
  using Configurations = std::vector<Configuration>;
  /**
   * @brief Result of an IK query: configurations with their residual errors and a message.
   *
   * The i-th residuals refer to the i-th configuration. Residual vectors may be empty
   * if the solver does not compute them.
   */
  struct Solutions : std::tuple<ik_solver::Configurations, std::vector<double>, std::vector<double>, std::string>
  {
    /**
     * @brief Joint configurations found.
     */
    const ik_solver::Configurations& configurations() const {return std::get<0>(*this);}
    ik_solver::Configurations& configurations() {return std::get<0>(*this);}
  
    /**
     * @brief Translation error of each configuration.
     */
    const std::vector<double>& translation_residuals() const {return std::get<1>(*this);}
    std::vector<double>& translation_residuals() {return std::get<1>(*this);}
  
    /**
     * @brief Rotation error of each configuration.
     */
    const std::vector<double>& rotation_residuals() const {return std::get<2>(*this);}
    std::vector<double>& rotation_residuals() {return std::get<2>(*this);}
  
    /**
     * @brief Optional message from the solver.
     */
    const std::string& message() const { return std::get<3>(*this); }
    std::string& message() { return std::get<3>(*this); }
  
    /**
     * @brief Remove all configurations, residuals and the message.
     */
    void clear() { configurations().clear(); translation_residuals().clear(); rotation_residuals().clear(); message().clear();}
  
  };
}

#endif //IK_SOLVER_CORE__INTERNAL__TYPES_H