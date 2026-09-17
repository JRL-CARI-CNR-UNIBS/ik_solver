# ik solver

The package is interconnected to the [`ik_solver_msgs`](https:/github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs.git) package. 

# Requirements

This packages require the utility set of packages [`cnr_common`](https://github.com/JRL-CARI-CNR-UNIBS/cnr_common). These libraries do not depend on ROS.

The environment variable `IK_SOLVER_LOGGER_CONFIG_PATH` should be set with the path to an appropriate `cnr_logger` configuration file. An example is provided [here](https://github.com/JRL-CARI-CNR-UNIBS/ik_solver/blob/master/ik_solver_core/config/default_logger.yaml).
In the future, this requirement will be removed.

# Usage

To simplify the setup process of multiple `ik_solver_node` servers, la `launch` file is provided. The launcher will load a `yaml` file containing the parameters as explained [below](#the-configuration-file) and will launch a solver server for each namespaced set of parameters.

To launch the ROS2 version

```bash
ros2 launch ik_solver ik_solver.launch.py config:=/path/to/config.yaml
```

A ROS1 launcher is provided but it is not stable yet.

# IK Solver ROS Node

The package provides a node named `ik_server_node` that exposes several services:
    
* `~get_ik` of type [`ik_solver_msgs/GetIk.srv`](https:/github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs/tree/parallel-ik/srv/GetIk.srv) that computes the set of feasible joint configurations corresponding to a single Cartesian pose. It depends on the server interface 

* `~get_ik_array` of type [`ik_solver_msgs/GetIkArray.srv`](https:/github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs/tree/parallel-ik/srv/GetIkArray.srv)
 that computes the set of feasible joint configurations corresponding to a set of Cartesian poses. 

* `~get_task_reduntant_ik_array` of type [`ik_solver_msgs/GetIkArray.srv`](https:/github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs/tree/parallel-ik/srv/GetIkArray.srv), same request/response as `~get_ik_array`. It perturbs each requested target pose (x/y/z/roll/pitch/yaw) according to the `task_reduntant` parameters (grid sweep or random sampling, see [the configuration file](#the-configuration-file)), solves IK for every perturbed pose, and returns the concatenation of all the resulting solutions.

* `~get_fk` of type [`ik_solver_msgs/GetFk.srv`](https:/github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs/tree/parallel-ik/srv/GetFk.srv) that computes the Cartesian pose corresponding to a single joint configuration. 

* `~get_fk_array` of type [`ik_solver_msgs/GetFkArray.srv`](https:/github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs/tree/parallel-ik/srv/GetFkArray.srv) that computes the Cartesian poses corresponding to a set of joint configurations. 

* `~get_bounds` of type [`ik_solver_msgs/GetBound.srv`](https:/github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs/tree/parallel-ik/srv/GetBound.srv) that get the SW boundaries of the joint ranges. 

* `~set_initial_configuration` of type `ik_solver_msgs/SetInitialConfiguration.srv` that sets or updates the home / initial joint configuration at runtime. When `filter_duplicates` is enabled in task-redundant IK, equivalent configurations modulo $2\pi$ are pruned, keeping the one closest in joint-space Euclidean distance to this initial configuration.

* `~reconfigure` of type `std_srvs/Trigger.srv` that allows to reconfigure the ik solver.
    
The node implements a pool of threads that offers access to the solver capabilities. 
The pool of thread dimension is set by the variable `MAX_NUM_PARALLEL_IK_SOLVER` in the `CMakeLists.txt`. 

## Packages structure

The framework is based on two packages:
* `ik_solver_core`, provide a ros-free core of the framework, such as the `IkSolverBase` abstract class which can be inherited to create ros-free plugins.
* `ik_solver`, is the ROS wrapper, compatible with both ROS1 and ROS2. Services and Nodes are defined here. Also, a convenience class `IKSolver`, inhereting `IkSolverBase`, is provided to simplify interfacing with `tf2`

## The IK Solver Plugin

The package implements an interface to a generic solver of the IK. The abstract class `IkSolverBase` is designed to allow an easy plugin inheritance.
The header file shows the basic functionalities of the plugin. It reads the URDF and extracts some useful information. Two abstract functions are then exposed for derived plugins.

The parameters that control the behavior of the class are loaded during the configuration. The configuration is done: 

* when the plugin is loaded.

* when the `~reconfigure` service is called.

A few parameters can be overridden by the service parameters (see below).

**Note:** If the plugin will be used with ROS, it is suggested to inherit from `IkSolver` instead of `IkSolverBase`.

Below the abstract base classes are reported. 

```cpp
/*
  ik_solver_core/include/ik_solver_core/ik_solver_base_class.h
*/

class IkSolverBase
{
public:
  IkSolverBase() = default;
  IkSolverBase(const IkSolverBase&) = delete;
  IkSolverBase(const IkSolverBase&&) = delete;
  IkSolverBase(IkSolverBase&&) = delete;
  virtual ~IkSolverBase() = default;

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

```cpp
/*
  ik_solver/include/ik_solver/ik_solver.hpp
*/

#if ROS_X == 1
  using Tf2BufferPtr = std::unique_ptr<tf2_ros::Buffer>;
#elif ROS_X == 2
  using Tf2BufferPtr = tf2_ros::Buffer::SharedPtr;
#endif

namespace ik_solver
{
class IkSolver : public IkSolverBase
{
protected:
  Tf2BufferPtr tf_buffer_;
public:
  IkSolver() : IkSolverBase() {}
  IkSolver(const IkSolver&) = delete;
  IkSolver(const IkSolver&&) = delete;
  IkSolver(IkSolver&&) = delete;
  virtual ~IkSolver() = default;

  void setBuffer(const tf2_ros::Buffer::SharedPtr& buffer){tf_buffer_=buffer;}

  virtual bool getTF(const std::string& a_name, const std::string& b_name, Eigen::Affine3d& T_ab) const;

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

# Every structure name is considered as the namespace of a new ik_solver node. Mulitple structure creates multiple nodes.
solver_namespace:

  # This param is inherited from the ik_solver_base class.
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

  # Duplicate filtering & initial configuration (task-redundant IK):
  # If filter_duplicates is true, solutions that represent the same physical robot pose
  # (modulo 2pi on revolute joints) are filtered out, keeping only the configuration
  # that minimizes Euclidean distance in joint space to initial_conf (or target seed).
  filter_duplicates: true
  initial_conf: [1.570796, -2.094395, 2.007129, -1.570796, -1.570796, 0.0] # e.g. [90°, -120°, 115°, -90°, -90°, 0°]
```

A runnable example, with two solvers and the `task_reduntant` parameters used by `~get_task_reduntant_ik_array`, is provided in [`ik_solver/examples/config.example.yaml`](ik_solver/examples/config.example.yaml):

```yaml
solver1:
  type: ik_solver/KukaIkSolver
  base_frame: base # base frame of the chain
  flange_frame: flange # end frame of the chain

  tool_frame: flange # destination frame of the IK (it should be rigid attached to flange_frame)
  desired_solutions: 32 # number of desired solution
  joint_names:
  - joint1
  - joint2
  - joint3
  - joint4
  - joint5
  - joint6

  # parameters used by the "get_task_reduntant_ik_array" service
  task_reduntant:
    type: grid # "grid" or "random"
    # grid mode: each axis is swept from min to max (inclusive) with the given step.
    # An axis that is not listed is not perturbed (single value 0).
    x:
      min: -0.01
      max: 0.01
      step: 0.01
    yaw:
      min: -0.5
      max: 0.5
      step: 0.25
    # random mode: each axis is sampled n_samples times from a normal distribution.
    # n_samples: 20
    # x:
    #   mean: 0.0
    #   standard_deviation: 0.01
    # yaw:
    #   mean: 0.0
    #   standard_deviation: 0.2

solver2:
  type: ik_solver/RosdynIkSolver
  base_frame: base # base frame of the chain
  flange_frame: flange # end frame of the chain

  tool_frame: flange # destination frame of the IK (it should be rigid attached to flange_frame)
  desired_solutions: 32 # number of desired solution
  joint_names:
  - joint1
  - joint2
  - joint3
  - joint4
  - joint5
  - joint6
```

# Testing (`ik_solver_test`)

The `ik_solver_test` package provides Python scripts to exercise a running `ik_solver` node, given its namespace (e.g. `solver1` from the configuration file above):

* `test_ik_fk.py` runs repeated FK→IK round-trips (`~get_fk` then `~get_ik`) on random valid joint configurations and reports IK residual errors, to sanity-check a solver plugin.

  ```bash
  ros2 run ik_solver_test test_ik_fk.py <namespace>
  ```

* `test_task_redundant_ik_array.py` computes a random target pose via `~get_fk`, calls `~get_task_reduntant_ik_array` on it, and logs the number of solutions and the max translation/rotation residual error found for each pose perturbation.

  ```bash
  ros2 run ik_solver_test test_task_redundant_ik_array.py <namespace>
  ```

* `visualize_task_redundant_ik_array.py` does the same call as above, then publishes each resulting joint configuration as a `moveit_msgs/DisplayRobotState` on `/display_robot_state`, one every `period_s` seconds (default `1.0`), so the redundant IK solutions can be visualized in RViz2 by adding a **RobotState** display (from `moveit_ros_visualization`) subscribed to that topic.

  ```bash
  ros2 run ik_solver_test visualize_task_redundant_ik_array.py <namespace> [period_s]
  ```

* `visualizer_script.py` (also available as `viusalizer_script.py`) calls the IK service sending a random joint configuration and displays the results on RViz. Supports both the base IK solver and task-redundant IK solver:
  - Each press of the **SPACE** button advances to the next solution configuration; once all solutions for the target are shown, pressing SPACE automatically requests a new random target configuration.
  - Press `n` or `r` to sample a new random target immediately.
  - Press `m` or `t` to toggle between **Base IK** and **Task-Redundant IK** dynamically at runtime.
  - Press `i` to set the currently displayed configuration as the new `initial_conf` on the IK server at runtime.

  ```bash
  # Task-redundant IK mode (default)
  ros2 run ik_solver visualizer_script.py [namespace]

  # Base IK mode
  ros2 run ik_solver visualizer_script.py [namespace] --base
  # or
  ros2 run ik_solver visualizer_script.py [namespace] --mode base

  # Optional: supply initial configuration to server on startup
  ros2 run ik_solver visualizer_script.py [namespace] --initial-conf 1.5708 -2.0944 2.0071 -1.5708 -1.5708 0.0
  ```


