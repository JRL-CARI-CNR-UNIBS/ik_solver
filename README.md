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

An example of a commented configuration template is provided in [`ik_solver_core/config/commented_params.yaml.template`](ik_solver_core/config/commented_params.yaml.template), and a runnable multi-solver example is in [`ik_solver/examples/config.example.yaml`](ik_solver/examples/config.example.yaml).

Every top-level key in the configuration YAML represents the namespace of an `ik_solver` node instance (e.g. `ur_ik_solver:` or `solver1:`). Multiple top-level blocks instantiate multiple independent solver servers.

### Configuration Parameters Reference

| Parameter | Type | Default | Description |
|---|---|---|---|
| `type` | string | *required* | The IK solver plugin to load (e.g. `ik_solver/Ur10eIkSolver`, `ik_solver/KukaIkSolver`, `ik_solver/RosdynIkSolver`). |
| `base_frame` | string | *required* | Base frame of the kinematic chain (e.g. `ur10e_base_link`, `base_link`, `world`). |
| `flange_frame` | string | *required* | Tip / end frame of the kinematic chain (e.g. `ur10e_tool0`, `flange`). |
| `tool_frame` | string | `flange_frame` | Destination tool frame of the IK request. Rigidly attached to `flange_frame` (e.g. `open_tip`). |
| `robot_description_topic` | string | `/robot_description` | Topic name for the URDF robot description. |
| `joint_names` | list[string] | *required* | Ordered list of joint names forming the kinematic chain. |
| `desired_solutions` | int | 32 | Number of desired IK solutions to compute. Can be overridden in service requests. |
| `min_stall_iterations` | int | 500 | Minimum stall iterations before solver aborts or switches. |
| `max_stall_iterations` | int | 3000 | Maximum stall iterations. |
| `parallel_ik_mode` | int | 2 | `0`: default, `1`: force parallelization across solver thread pool, `2`: disable parallelization. |
| `update_recursively_seeds` | int | 1 | `0`: default, `1`: force update seeds using solutions from previous pose, `2`: disable seed update. Ignored if `parallel_ik_mode` is `2`. |
| `filter_duplicates` | bool | `true` | When `true`, filters out physically identical configurations (modulo $2\pi$ on revolute joints) generated by task-redundant IK. |
| `initial_conf` | list[float] | `[]` | Robot home/initial joint configuration in radians (e.g. `[1.570796, -2.094395, 2.007129, -1.570796, -1.570796, 0.0]`). When `filter_duplicates` is active, the solver retains the duplicate configuration that minimizes joint-space Euclidean distance to `initial_conf` (or target seed). Can be updated at runtime via the `~set_initial_configuration` service. |
| `task_reduntant` | mapping | *optional* | Perturbation sampling settings used by the `~get_task_reduntant_ik_array` service. |

### Duplicate Filtering & Initial Configuration

For robots with wide revolute joint limits (such as the UR10e with $[-2\pi, 2\pi]$ limits on wrists), the solver can return $2^3 = 8$ mathematically distinct configurations that place the robot in the exact same physical posture.
- **`filter_duplicates: true`**: Identifies all duplicate solutions differing by integer multiples of $2\pi$ on revolute joints.
- **`initial_conf`**: Specifies the reference joint configuration $[q_1, \dots, q_n]$ in radians.
- For each group of duplicate configurations, the solver computes the joint-space Euclidean distance:
  $$\| q_{\text{cand}} - q_{\text{ref}} \|_2 = \sqrt{\sum_{i} (q_{\text{cand}, i} - q_{\text{ref}, i})^2}$$
  where $q_{\text{ref}}$ is the target seed (if provided) or `initial_conf`. Only the closest configuration is returned, preventing redundant collision checks and avoiding unnecessary $360^\circ$ joint unwinds.
- The initial configuration can be dynamically updated at runtime using the `~set_initial_configuration` service.

### Task-Redundant IK Perturbations (`task_reduntant`)

The `~get_task_reduntant_ik_array` service applies perturbations around the target tool frame before solving IK. Two sampling strategies are supported:

#### 1. Random Sampling (`type: random`)
Samples $N$ perturbations from normal distributions across selected axes. Axes not specified are kept unperturbed ($0$).
```yaml
task_reduntant:
  type: random
  n_samples: 450                 # Total perturbation count
  seed: 42                       # Optional pseudo-random seed for reproducibility
  roll:
    mean: 0.261799               # Mean offset (radians)
    standard_deviation: 1.439897 # Standard deviation (radians)
  pitch:
    mean: 0.0
    standard_deviation: 0.392699
  yaw:
    mean: 0.0
    standard_deviation: 0.392699
  # Translation axes can also be perturbed (in meters):
  # x:
  #   mean: 0.0
  #   standard_deviation: 0.01
```

#### 2. Grid Sweep (`type: grid`)
Sweeps perturbations across a regular Cartesian grid spanning specified axes:
```yaml
task_reduntant:
  type: grid
  roll:
    min: -2.617994       # Lower bound (rad)
    max: 3.141593        # Upper bound (rad)
    step: 0.523599       # Step size (rad)
  pitch:
    min: -0.785398
    max: 0.785398
    step: 0.392699
  yaw:
    min: -0.785398
    max: 0.785398
    step: 0.392699
```

### Complete Configuration Example

Below is a complete configuration example for a UR10e cell (as in `ply_publisher/config/ik_params.yaml`):

```yaml
ur_ik_solver:
  type: ik_solver/Ur10eIkSolver
  base_frame: ur10e_base_link
  flange_frame: ur10e_tool0
  tool_frame: open_tip
  robot_description_topic: /robot_description
  desired_solutions: 256

  joint_names:
    - ur10e_shoulder_pan_joint
    - ur10e_shoulder_lift_joint
    - ur10e_elbow_joint
    - ur10e_wrist_1_joint
    - ur10e_wrist_2_joint
    - ur10e_wrist_3_joint

  min_stall_iterations: 100
  max_stall_iterations: 100000
  parallel_ik_mode: 2

  # Duplicate filtering & initial configuration
  filter_duplicates: true
  initial_conf: [1.57079632679, -2.09439510239, 2.00712863979, -1.57079632679, -1.57079632679, 0.0] # [90°, -120°, 115°, -90°, -90°, 0°]

  # Task-redundant IK perturbation sampling
  task_reduntant:
    type: random
    n_samples: 450
    seed: 42
    roll:
      mean: 0.26179938780
      standard_deviation: 1.43989663290
    pitch:
      mean: 0.0
      standard_deviation: 0.3926990817
    yaw:
      mean: 0.0
      standard_deviation: 0.3926990817
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


