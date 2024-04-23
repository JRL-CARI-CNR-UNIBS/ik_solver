# ik solver

The package is interconnected to the [`ik_solver_msgs``](https:/github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs.git) package. 

# IK Solver ROS Node

The package provides a node named `ik_server_node` that exposes several services:
    
* `~get_ik` of type [`ik_solver_msgs/GetIk.srv`](https:/github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs/tree/parallel-ik/srv/GetIk.srv) that computes the set of feasible joint configurations corresponding to a single Cartesian pose. It depends on the server interface 

* `~get_ik_array` of type [`ik_solver_msgs/GetIkArray.srv`](https:/github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs/tree/parallel-ik/srv/GetIkArray.srv)
 that computes the set of feasible joint configurations corresponding to a set of Cartesian poses. 

* `~get_fk` of type [`ik_solver_msgs/GetFk.srv`](https:/github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs/tree/parallel-ik/srv/GetFk.srv) that computes the Cartesian pose corresponding to a single joint configuration. 

* `~get_fk_array` of type [`ik_solver_msgs/GetFkArray.srv`](https:/github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs/tree/parallel-ik/srv/GetFkArray.srv) that computes the Cartesian poses corresponding to a set of joint configurations. 

* `~get_bounds` of type [`ik_solver_msgs/GetBound.srv`](https:/github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs/tree/parallel-ik/srv/GetBound.srv) that get the SW boundaries of the joint ranges. 

* `~reconfigure` of type `std_srvs/Trigger.srv` that allows to reconfigure the ik solver.
    
The node implements a pool of threads that offers access to the solver capabilities. 
The pool of thread dimension is set by the variable `MAX_NUM_PARALLEL_IK_SOLVER` in the `CMakeLists.txt`. 

## The IK Solver Plugin

The package implements an interface to a generic solver of the IK. The abstract class `IkSolver` is designed to allow an easy plugin inheritance.
The header file shows the basic functionalities of the plugin. It reads the URDF and extracts some useful information. Two abstract functions are then exposed for derived plugins.

The parameters that control the behavior of the class are loaded during the configuration. The configuration is done: 

* when the plugin is loaded.

* when the `~reconfigure` service is called.

A few parameters can be overridden by the service parameters (see below).

Below the abstract base class is reported. 

```cpp
class IkSolver
{
public:
  IkSolver() = default;
  IkSolver(const IkSolver&) = delete;
  IkSolver(const IkSolver&&) = delete;
  IkSolver(IkSolver&&) = delete;
  virtual ~IkSolver() = default;

  virtual bool config(const ros::NodeHandle& nh, const std::string& param_ns = "");

  // FK flange to base
  virtual Solutions getIk(const Eigen::Affine3d& T_base_flange, const Configurations& seeds,
                          const int& desired_solutions = -1, const int& min_stall_iterations = -1,
                          const int& max_stall_iterations = -1) = 0;

  // FK base to flange
  virtual Eigen::Affine3d getFK(const Configuration& s) = 0;

  const std::vector<std::string>& joint_names() const;
  const std::string& base_frame() const;
  const std::string& flange_frame() const;
  const std::string& tool_frame() const;
  const Eigen::Affine3d& transform_from_flange_to_tool() const;
  Eigen::Affine3d transform_from_tool_to_flange() const;
  const Configuration& lb() const;
  const Configuration& ub() const;
  const std::vector<bool>& revolute() const;
  const int& min_stall_iterations() const;
  const int& max_stall_iterations() const;
  const int& desired_solutions() const;
  const int& parallelize() const;
  const std::string param_namespace() const;

protected:
  std::string params_ns_;
  ros::NodeHandle robot_nh_;

  Eigen::Affine3d T_tool_flange_;
  tf::TransformListener listener_;
  std::string base_frame_;
  std::string flange_frame_;
  std::string tool_frame_;
  std::vector<std::string> joint_names_;

  Configuration ub_;
  Configuration lb_;
  std::vector<bool> revolute_;
  int min_stall_iter_ = 998;
  int max_stall_iter_ = 999;
  int max_iter_ = 1000000;
  int desired_solutions_ = 8;
  int parallelize_ = 0;
  int exploit_solutions_as_seed_ = 0;

  urdf::Model model_;

  bool getFlangeTool();
};
```


## The configuration file

An example of a configuration file with the parameters needed by the plugin and the node is [here](config/params.yaml.template).

```yaml
###################################################
#
# The param needed by the every plugin that inherits 
# from ik_solver base class
#
###################################################

# This param is inherited from the ik_solver base class.
# This param tells the plugin loader to load the 
# SPECIFIC_PLUGIN
type: ik_solver/SPECIFIC_PLUGIN

# Parameters used by the ik_solver node. The node creates a number of parallel threads for
# speed up the IK and FK computation.
# If the parallel mode is active, the speed is very high, but the previous IK solution cannot be exploited
# If the parallel mode is off, you can select if the previous IK solution is used as seed for the next IK solution
# or not
# NOTE:
#   parallel_ik_mode and update_recursively_seeds are the default value. 
#   These values can be overridden using the service messages each time the service are called.
#   
parallel_ik_mode: 2           # 0 default, 1 force parallelization, 2 disable parallelization
update_recursively_seeds: 1   # 0 default, 1 force update, 2 disable update !!! If paralle_ik_mode is 2, this is neglected

# Parameters inherited from the base class
# NOTE: here the parameter are for the whole chain, i.e., both axis and robot arm chain
group_name: manipulator
base_frame: world             # base frame of the chain
flange_frame: flange          # end frame of the chain
desired_solutions: 32         # number of desired solution
                              # This parameter is overridden by the max_number_of_solutions in the GetIk service if it is different from 0
joint_names:                  # name of the whole chain 
- joint_7
- joint_1
- joint_2
- joint_3
- joint_4
- joint_5
- joint_6

min_stall_iterations: 500      # This parameter is overridden by the stall_iterations in the GetIk service if it is different from 0
max_stall_iterations: 3000
```
