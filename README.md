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

* `~get_task_redundant_ik_array` of type [`ik_solver_msgs/GetIkArray.srv`](https:/github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs/tree/parallel-ik/srv/GetIkArray.srv), same request/response as `~get_ik_array`. It perturbs each requested target pose (x/y/z/roll/pitch/yaw) according to the `task_redundant` parameters (grid sweep or random sampling, see [the configuration file](#the-configuration-file)), solves IK for every perturbed pose, and returns the concatenation of all the resulting solutions.

* `~get_fk` of type [`ik_solver_msgs/GetFk.srv`](https:/github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs/tree/parallel-ik/srv/GetFk.srv) that computes the Cartesian pose corresponding to a single joint configuration. 

* `~get_fk_array` of type [`ik_solver_msgs/GetFkArray.srv`](https:/github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs/tree/parallel-ik/srv/GetFkArray.srv) that computes the Cartesian poses corresponding to a set of joint configurations. 

* `~get_bounds` of type [`ik_solver_msgs/GetBound.srv`](https:/github.com/JRL-CARI-CNR-UNIBS/ik_solver_msgs/tree/parallel-ik/srv/GetBound.srv) that get the SW boundaries of the joint ranges. 

* `~set_bounds` of type `ik_solver_msgs/SetBound.srv` that changes the SW boundaries of the listed joints at runtime. The limits are clamped to the URDF ones, the response contains the limits in use, and `~reconfigure` restores the limits of the parameters.

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
  bool setBounds(const Configuration& lb, const Configuration& ub);  // clamped to the URDF limits
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
| `limits/<joint>/lower`, `limits/<joint>/upper` | float | URDF limits | Restrict the URDF joint limits (a value wider than the URDF one is ignored). Can be changed at runtime via the `~set_bounds` service; `~reconfigure` restores these values. |
| `desired_solutions` | int | 32 | Number of desired IK solutions to compute. Can be overridden in service requests. |
| `min_stall_iterations` | int | 500 | Minimum stall iterations before solver aborts or switches. |
| `max_stall_iterations` | int | 3000 | Maximum stall iterations. |
| `parallel_ik_mode` | int | 2 | `0`: default, `1`: force parallelization across solver thread pool, `2`: disable parallelization. |
| `update_recursively_seeds` | int | 1 | `0`: default, `1`: force update seeds using solutions from previous pose, `2`: disable seed update. Ignored if `parallel_ik_mode` is `2`. |
| `filter_duplicates` | bool | `true` | When `true`, filters out physically identical configurations (modulo $2\pi$ on revolute joints) generated by task-redundant IK. |
| `initial_conf` | list[float] | `[]` | Robot home/initial joint configuration in radians (e.g. `[1.570796, -2.094395, 2.007129, -1.570796, -1.570796, 0.0]`). When `filter_duplicates` is active, the solver retains the duplicate configuration that minimizes joint-space Euclidean distance to `initial_conf` (or target seed). Can be updated at runtime via the `~set_initial_configuration` service. |
| `task_redundant` | mapping | *optional* | Perturbation sampling settings used by the `~get_task_redundant_ik_array` service. The old misspelled name `task_reduntant` is still accepted. |

### Duplicate Filtering & Initial Configuration

For robots with wide revolute joint limits (such as the UR10e with $[-2\pi, 2\pi]$ limits on wrists), the solver can return $2^3 = 8$ mathematically distinct configurations that place the robot in the exact same physical posture.
- **`filter_duplicates: true`**: Identifies all duplicate solutions differing by integer multiples of $2\pi$ on revolute joints.
- **`initial_conf`**: Specifies the reference joint configuration $[q_1, \dots, q_n]$ in radians.
- For each group of duplicate configurations, the solver computes the joint-space Euclidean distance:
  $$\| q_{\text{cand}} - q_{\text{ref}} \|_2 = \sqrt{\sum_{i} (q_{\text{cand}, i} - q_{\text{ref}, i})^2}$$
  where $q_{\text{ref}}$ is the target seed (if provided) or `initial_conf`. Only the closest configuration is returned, preventing redundant collision checks and avoiding unnecessary $360^\circ$ joint unwinds.
- The initial configuration can be dynamically updated at runtime using the `~set_initial_configuration` service.

### Task-Redundant IK Perturbations (`task_redundant`)

The `~get_task_redundant_ik_array` service applies perturbations around the target tool frame before solving IK. Two sampling strategies are supported:

#### 1. Random Sampling (`type: random`)
Samples $N$ perturbations from normal distributions across selected axes. Axes not specified are kept unperturbed ($0$).
```yaml
task_redundant:
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
task_redundant:
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

Below is a complete configuration example for a UR10e cell. A runnable example, with two solvers, is also provided in [`ik_solver/examples/config.example.yaml`](ik_solver/examples/config.example.yaml):

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
  task_redundant:
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

* `test_task_redundant_ik_array.py` computes a random target pose via `~get_fk`, calls `~get_task_redundant_ik_array` on it, and logs the number of solutions and the max translation/rotation residual error found for each pose perturbation.

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

* `test_service_timing.py` measures the call time of each service as seen by the client (serialization, transport, computation and response) and prints mean, std, min, median, p95 and max. Single services (`get_fk`, `get_ik`) are called `--trials` times; array services are called `--repetitions` times for each `--payloads` size. `--parallelize {default,force,disable}` and `--threads N` set the `parallelize` and `num_threads` fields of the IK array requests; every combination given is run. `--csv` saves the statistics.

  ```bash
  ros2 run ik_solver_test test_service_timing.py <namespace>
  ros2 run ik_solver_test test_service_timing.py <namespace> --services get_ik_array --parallelize force disable
  ros2 run ik_solver_test test_service_timing.py <namespace> --services get_ik_array --parallelize force --threads 1 2 4 8
  ```

## Benchmark

Results of `test_service_timing.py` with the default options (10 repetitions per payload, no seeds), Release build.

**Machine:** Lenovo laptop (21A4), AMD Ryzen 5 5600U (6 cores / 12 threads, up to 4.3 GHz), 35 GB RAM, Ubuntu 24.04, ROS 2 Jazzy, Fast DDS (`rmw_fastrtps_cpp`, default), GCC 13.3, `balanced` power profile.
**Solver:** UR10e analytic solver (`ur_ik_solver`, `Ur10eIkSolver`); `MAX_NUM_PARALLEL_IK_SOLVER` = 10.

Mean call time of the IK array services, single-thread (`parallel_ik_mode: 2`) vs multi-thread (`parallelize: force`, 10 threads):

| Service | Payload | Single-thread [ms] | Multi-thread [ms] | Speed-up |
|---|---:|---:|---:|---:|
| `get_ik_array` | 10 | 10.0 | 5.7 | 1.7× |
| `get_ik_array` | 100 | 64.9 | 25.2 | 2.6× |
| `get_ik_array` | 1000 | 652.2 | 263.4 | 2.5× |
| `get_ik_array` | 10000 | 7009.1 | 2799.1 | 2.5× |
| `get_task_redundant_ik_array` | 10 | 31.4 | 11.5 | 2.7× |
| `get_task_redundant_ik_array` | 100 | 278.2 | 87.2 | 3.2× |
| `get_task_redundant_ik_array` | 1000 | 2861.0 | 872.0 | 3.3× |
| `get_task_redundant_ik_array` | 10000 | 32406.6 | 9523.8 | 3.4× |

The speed-up is below the number of threads: the CPU has 6 physical cores, and the request/response transport and the Python (de)serialization in the client are not parallelized. `get_fk`, `get_ik` and `get_fk_array` do not depend on the parallel mode (`get_fk_array` is always multi-thread); differences between the two runs are run-to-run variation.

<details>
<summary>Full results</summary>

Single-thread (`parallel_ik_mode: 2`):

```
service                       payload     n        mean         std         min      median         p95         max    per item    fail
---------------------------------------------------------------------------------------------------------------------------------------
get_fk                              1  1000       0.222       0.057       0.181       0.210       0.274       1.270      0.2223       0
get_ik                              1  1000       1.255      10.560       0.306       0.834       1.677     334.635      1.2548       0
get_fk_array                       10    10      34.556     105.248       0.870       1.213     184.878     334.093      3.4556       0
get_fk_array                      100    10       1.952       0.461       1.485       1.822       2.652       2.737      0.0195       0
get_fk_array                     1000    10      13.702       7.726       9.998      11.285      25.371      35.517      0.0137       0
get_fk_array                    10000    10     167.564      89.527     128.698     140.595     299.147     421.514      0.0168       0
get_ik_array                       10    10      10.009       2.735       7.019       9.402      14.126      15.104      1.0009       0
get_ik_array                      100    10      64.861       4.751      59.428      64.540      72.899      75.482      0.6486       0
get_ik_array                     1000    10     652.198      15.766     627.115     655.417     669.821     670.783      0.6522       0
get_ik_array                    10000    10    7009.102     136.464    6863.113    6957.403    7236.392    7322.716      0.7009       0
get_task_redundant_ik_array        10    10      31.446       5.299      24.186      30.988      39.222      40.139      3.1446       0
get_task_redundant_ik_array       100    10     278.184      15.995     259.645     276.940     302.236     315.333      2.7818       0
get_task_redundant_ik_array      1000    10    2861.006      62.599    2773.151    2866.295    2956.510    2995.925      2.8610       0
get_task_redundant_ik_array     10000    10   32406.561     689.166   31238.897   32225.405   33392.875   33527.086      3.2407       1
```

Multi-thread (`--parallelize force`, 10 threads):

```
service                          mode  payload     n        mean         std         min      median         p95         max    per item    fail
------------------------------------------------------------------------------------------------------------------------------------------------
get_fk                              -        1  1000       1.143       0.171       0.943       1.125       1.340       4.778      1.1427       0
get_ik                              -        1  1000       1.834       0.396       1.091       1.765       2.565       4.257      1.8345       0
get_fk_array                        -       10    10       2.367       0.525       1.743       2.621       2.963       3.071      0.2367       0
get_fk_array                        -      100    10       3.555       0.614       2.626       3.680       4.327       4.519      0.0356       0
get_fk_array                        -     1000    10      14.417       5.336      11.844      12.720      22.378      29.509      0.0144       0
get_fk_array                        -    10000    10     135.227       8.382     124.790     132.266     147.603     149.031      0.0135       0
get_ik_array                    force       10    10       5.734       0.936       4.077       5.843       6.867       6.895      0.5734       0
get_ik_array                    force      100    10      25.180       2.175      21.990      24.473      28.626      28.920      0.2518       0
get_ik_array                    force     1000    10     263.370       6.833     251.019     265.445     270.527     270.789      0.2634       0
get_ik_array                    force    10000    10    2799.074     133.242    2651.594    2758.381    3012.588    3061.752      0.2799       2
get_task_redundant_ik_array     force       10    10      11.523       1.095       9.884      11.338      12.724      12.728      1.1523       0
get_task_redundant_ik_array     force      100    10      87.221       5.020      80.771      86.652      95.046      97.465      0.8722       0
get_task_redundant_ik_array     force     1000    10     871.965      19.359     839.280     872.820     896.215     904.675      0.8720       0
get_task_redundant_ik_array     force    10000    10    9523.793     231.736    9228.702    9502.551    9879.866   10013.918      0.9524       0
```

</details>

# Contributors

* Manuel Beschi
* Nicola Pedrocchi
* Roberto Fausti
* Federico Parma



# Tutorial and API documentation

* [docs/TUTORIAL.md](docs/TUTORIAL.md): start a server, query it from the command line and from Python, use the task-redundant IK, and write a new solver plugin.
* [docs/API.md](docs/API.md): API reference of the classes, functions and scripts (generated from the Doxygen comments).
* The code is documented with Doxygen. From the repository folder run `doxygen Doxyfile` and open `doxygen/html/index.html`.
