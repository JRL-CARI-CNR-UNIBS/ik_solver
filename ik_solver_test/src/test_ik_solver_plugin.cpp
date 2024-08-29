#include "ik_solver_test/test_ik_solver_plugin.hpp"
#include <pluginlib/class_list_macros.hpp>


namespace test_ik_solver
{
  bool TestIkSolver::config(const std::string& param_ns)
  {
    RCLCPP_INFO(this->logger_, "Configuration step");
    RCLCPP_INFO(this->logger_, "Parameters namespace: %s", param_ns.c_str());

    return true;
  }

  Solutions TestIkSolver::getIk(const Eigen::Affine3d& T_base_flange, const Configurations& seeds, const int& desired_solutions, const int& min_stall_iterations, const int& max_stall_iterations)
  {
    RCLCPP_INFO(this->logger_, "getIk() function called");
    RCLCPP_INFO(this->logger_, "Got the following arguments:");
    RCLCPP_INFO(this->logger_, "> T_base_flange:\n%.2f%.2f%.2f%.2f\n%.2f%.2f%.2f%.2f\n%.2f%.2f%.2f%.2f\n%.2f%.2f%.2f%.2f",
        T_base_flange.matrix()(0,0), T_base_flange.matrix()(0,1), T_base_flange.matrix()(0,2), T_base_flange.matrix()(0,3),
        T_base_flange.matrix()(1,0), T_base_flange.matrix()(1,1), T_base_flange.matrix()(1,2), T_base_flange.matrix()(1,3),
        T_base_flange.matrix()(2,0), T_base_flange.matrix()(2,1), T_base_flange.matrix()(2,2), T_base_flange.matrix()(2,3),
        T_base_flange.matrix()(3,0), T_base_flange.matrix()(3,1), T_base_flange.matrix()(3,2), T_base_flange.matrix()(3,3));
    RCLCPP_INFO(this->logger_, "> seeds:");
    for(auto conf : seeds)
    {
      RCLCPP_INFO(this->logger_, ">> seed->dimension: %ld", conf.rows());
    }
    RCLCPP_INFO(this->logger_, "> Number of desired solutions: %d", desired_solutions);
    RCLCPP_INFO(this->logger_, "> Stall iterations: min (%d), max (%d)", min_stall_iterations, max_stall_iterations);

    Solutions sol;
    sol.clear();
    sol.configurations().push_back({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    RCLCPP_INFO(this->logger_, "< Output solution configuration: {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}");
    return sol;
  }

  Eigen::Affine3d TestIkSolver::getFK(const Configuration& s)
  {
    RCLCPP_INFO(this->logger_, "getFk() function called");
    RCLCPP_INFO(this->logger_, "> Input configuration:");
    for(auto v : s)
    {
      RCLCPP_INFO(this->logger_, ">> %.2f", v);
    }
    RCLCPP_INFO(this->logger_, "< Return identity affine matrix");
    return Eigen::Affine3d::Identity();
  }
}

PLUGINLIB_EXPORT_CLASS(test_ik_solver::TestIkSolver, ik_solver::IkSolver);
