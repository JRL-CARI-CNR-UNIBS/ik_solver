#ifndef TEST_IK_SOLVER_PLUGIN_HPP
#define TEST_IK_SOLVER_PLUGIN_HPP

#include "ik_solver_core/ik_solver_base_class.h"

#include <rclcpp/rclcpp.hpp>

namespace test_ik_solver
{
using namespace ik_solver;

class TestIkSolver : public ik_solver::IkSolver
{
public:
  TestIkSolver() : logger_(rclcpp::get_logger("TestIkSolver")){}
  virtual bool config(const std::string& param_ns = "") override;
  virtual Solutions getIk(const Eigen::Affine3d& T_base_flange, const Configurations& seeds, const int& desired_solutions = -1, const int& min_stall_iterations = -1, const int& max_stall_iterations = -1) override;
  virtual Eigen::Affine3d getFK(const Configuration& s) override;


private:
  rclcpp::Logger logger_;
};
}

#endif // TEST_IK_SOLVER_PLUGIN_HPP
