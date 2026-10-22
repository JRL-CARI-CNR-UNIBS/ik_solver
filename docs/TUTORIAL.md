# ik_solver tutorial

This tutorial covers the three things you usually do with `ik_solver`:

1. start an IK server for a robot;
2. query it (IK, FK, task-redundant IK) from the command line and from Python;
3. write a new solver plugin.

The examples use a UR10e with the `ur_ik_solver` plugin. Change frames and joint names to match your
robot.

## Prerequisites

- `ik_solver`, `ik_solver_core` and `ik_solver_msgs` built in your workspace;
- a solver plugin for your robot (for UR robots: `ur_ik_solver`);
- `cnr_param` and `cnr_logger`, and the environment variables they need:

```bash
# cnr_logger configuration used by the solvers
export IK_SOLVER_LOGGER_CONFIG_PATH=$(ros2 pkg prefix ik_solver_core)/share/ik_solver_core/config/default_logger.yaml
# folder used by cnr_param to store the parameters
export CNR_PARAM_ROOT_DIRECTORY=/tmp/cnr_param
```

If `IK_SOLVER_LOGGER_CONFIG_PATH` is not set, the solver configuration fails with
`Missing environemnt variable IK_SOLVER_LOGGER_CONFIG_PATH!`.

## 1. Start an IK server

### 1.1 Write the configuration

Each top-level key of the configuration file creates one IK server node with that namespace. Create
`my_ik.yaml`:

```yaml
ur10e_ik:
  type: ik_solver/Ur10eIkSolver        # plugin name, as in the plugin XML file
  base_frame: base_link                 # first frame of the chain
  flange_frame: tool0                   # last frame of the chain
  tool_frame: tool0                     # frame used for the target poses (rigidly attached to the flange)
  robot_description_topic: /robot_description
  joint_names:
    - shoulder_pan_joint
    - shoulder_lift_joint
    - elbow_joint
    - wrist_1_joint
    - wrist_2_joint
    - wrist_3_joint
  desired_solutions: 32
  min_stall_iterations: 100
  max_stall_iterations: 100000
  parallel_ik_mode: 2                   # 2 = sequential by default; requests can override it
```

The joint limits are read from the URDF. You can restrict them further:

```yaml
  limits:
    shoulder_pan_joint:
      lower: -3.14
      upper: 3.14
```

### 1.2 Launch

The robot description and the TF tree must be available (e.g. launch your robot with
`robot_state_publisher` first). Then:

```bash
ros2 launch ik_solver ik_solver.launch.py config:=$(pwd)/my_ik.yaml
```

The launch file loads the YAML into `cnr_param` and starts one `ik_solver_node` per top-level key. When the
node is ready it prints:

```
/ur10e_ik (type ik_solver/Ur10eIkSolver) is ready to compute IK
IK Solver node: READY
```

If it keeps printing `Waiting for configuration. Probably robot_description is missing`, check that the
robot description is published on `robot_description_topic`.

## 2. Query the server

The node offers these services in its namespace:

| Service | Type | What it does |
|---|---|---|
| `get_ik` | `ik_solver_msgs/srv/GetIk` | IK of one tool pose |
| `get_ik_array` | `ik_solver_msgs/srv/GetIkArray` | IK of many tool poses |
| `get_task_redundant_ik_array` | `ik_solver_msgs/srv/GetIkArray` | IK of many poses, each perturbed as set in `task_redundant` |
| `get_fk` | `ik_solver_msgs/srv/GetFk` | pose of the tool (or of `tip_frame`) for one configuration |
| `get_fk_array` | `ik_solver_msgs/srv/GetFkArray` | the same for many configurations |
| `get_bounds` | `ik_solver_msgs/srv/GetBound` | joint limits |
| `get_frames` | `ik_solver_msgs/srv/GetFrames` | base, flange and tool frames |
| `change_tool` | `ik_solver_msgs/srv/ChangeTool` | change the tool frame |
| `set_initial_configuration` | `ik_solver_msgs/srv/SetInitialConfiguration` | reference configuration for duplicate filtering |
| `reconfigure` | `std_srvs/srv/Trigger` | reload the parameters |

### 2.1 From the command line

Frames and limits:

```bash
ros2 service call /ur10e_ik/get_frames ik_solver_msgs/srv/GetFrames {}
ros2 service call /ur10e_ik/get_bounds ik_solver_msgs/srv/GetBound {}
```

IK of a tool pose expressed in `base_link`:

```bash
ros2 service call /ur10e_ik/get_ik ik_solver_msgs/srv/GetIk "{
  target: {pose: {header: {frame_id: base_link},
                  pose: {position: {x: 0.6, y: 0.2, z: 0.4},
                         orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}},
  max_number_of_solutions: 8}"
```

The pose can be expressed in any frame known to TF: the server converts it to the base frame.
`max_number_of_solutions: 0` and `stall_iterations: 0` mean "use the configured value".

### 2.2 From Python: FK → IK round trip

```python
import rclpy
from ik_solver_msgs.srv import GetFk, GetIk
from ik_solver_msgs.msg import Configuration

rclpy.init()
node = rclpy.create_node("ik_client")
fk_cli = node.create_client(GetFk, "/ur10e_ik/get_fk")
ik_cli = node.create_client(GetIk, "/ur10e_ik/get_ik")
fk_cli.wait_for_service()
ik_cli.wait_for_service()

joints = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
q = [0.3, -1.2, 1.4, -1.7, -1.57, 0.0]

# FK: tool pose of q, expressed in base_link
fk_req = GetFk.Request(joint_names=joints, reference_frame="base_link",
                       configuration=Configuration(configuration=q))
fut = fk_cli.call_async(fk_req)
rclpy.spin_until_future_complete(node, fut)
pose = fut.result().pose

# IK of that pose: q should be among the solutions
ik_req = GetIk.Request()
ik_req.target.pose = pose
fut = ik_cli.call_async(ik_req)
rclpy.spin_until_future_complete(node, fut)
res = fut.result()
print(f"{len(res.solution.configurations)} solutions, joint order {list(res.joint_names)}")
for sol, et, er in zip(res.solution.configurations,
                       res.solution.translation_residual_errors,
                       res.solution.rotation_residual_errors):
    print([round(v, 3) for v in sol.configuration], f"err: {et:.1e} m, {er:.1e} rad")
```

The solutions are always in the order of `res.joint_names` (the order of the `joint_names` parameter).
Seeds (`target.seeds`) can be given in a different order if you set `seed_joint_names`.

`ik_solver_test/scripts/test_ik_fk.py` runs this round trip on random configurations:

```bash
ros2 run ik_solver_test test_ik_fk.py ur10e_ik
```

### 2.3 IK of many poses

Use `get_ik_array` with one `IkTarget` per pose. The `parallelize` field selects the mode:

- sequential: the solutions of each pose are used as seeds for the next one. This is good for a sequence of
  close poses, such as a path;
- parallel: poses are solved at the same time by `MAX_NUM_PARALLEL_IK_SOLVER` solver instances (by default,
  the number of CPU cores of the build machine minus 2). Set `num_threads` to use fewer threads
  (`0` uses all of them).

### 2.4 Task-redundant IK

Many tasks do not constrain all six degrees of freedom of the tool: for example, a drilling or a
spraying tool can rotate around its axis. `get_task_redundant_ik_array` solves the IK of each target
pose perturbed by a set of small transforms, and returns all the solutions together.

Add the perturbations to the configuration. For example, to allow any rotation about the tool z axis
in 10° steps:

```yaml
  task_redundant:
    type: grid
    yaw:
      min: -3.14159
      max: 3.14159
      step: 0.17453
  filter_duplicates: true        # remove equivalent configurations
  initial_conf: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]  # among duplicates, keep the closest to this one
```

Or sample them randomly:

```yaml
  task_redundant:
    type: random
    n_samples: 200
    seed: 42                     # optional, for reproducible results
    yaw:
      mean: 0.0
      standard_deviation: 0.5
```

Axes not listed are not perturbed. The perturbation is applied in the tool frame (`T_target * T_perturbation`).
Then call `reconfigure` or restart the node, and try it:

```bash
ros2 run ik_solver_test test_task_redundant_ik_array.py ur10e_ik
ros2 run ik_solver_test visualize_task_redundant_ik_array.py ur10e_ik 0.5   # show the solutions in RViz
```

## 3. Write a solver plugin

A plugin derives from `ik_solver::IkSolver` and implements three functions.

### 3.1 Header

`include/my_ik_solver/my_ik_solver.hpp`:

```cpp
#pragma once
#include <ik_solver/ik_solver.hpp>

namespace my_ik_solver
{
class MyIkSolver : public ik_solver::IkSolver
{
public:
  bool config(const std::string& param_ns = "") override;

  ik_solver::Solutions getIk(const Eigen::Affine3d& T_base_flange, const ik_solver::Configurations& seeds,
                             const int& desired_solutions = -1, const int& min_stall_iterations = -1,
                             const int& max_stall_iterations = -1) override;

  Eigen::Affine3d getFK(const ik_solver::Configuration& q) override;

private:
  double my_param_ = 0.0;
};
}  // namespace my_ik_solver
```

### 3.2 Implementation

`src/my_ik_solver.cpp`:

```cpp
#include <my_ik_solver/my_ik_solver.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace my_ik_solver
{
bool MyIkSolver::config(const std::string& param_ns)
{
  // Reads frames, joint names, limits, desired_solutions, ... (see IkSolverBase)
  if (!IkSolver::config(param_ns))
    return false;

  // Your own parameters, under the same namespace
  std::string what;
  if (!cnr::param::get(params_ns_ + "my_param", my_param_, what))
    my_param_ = 1.0;
  return true;
}

ik_solver::Solutions MyIkSolver::getIk(const Eigen::Affine3d& T_base_flange, const ik_solver::Configurations& seeds,
                                       const int& desired_solutions, const int& min_stall_iterations,
                                       const int& max_stall_iterations)
{
  const int n_desired = desired_solutions > 0 ? desired_solutions : desired_solutions_;

  ik_solver::Solutions sol;
  // ... compute the configurations that bring the flange to T_base_flange ...
  // Keep only configurations inside [lb_, ub_]; ik_solver::outOfBound() helps.
  // For revolute joints, ik_solver::getMultiplicity() adds the q +/- 2*k*pi equivalents.
  for (const ik_solver::Configuration& q : my_analytic_solutions)
  {
    if (!ik_solver::outOfBound(q, ub_, lb_).empty())
      continue;
    sol.configurations().push_back(q);
    sol.translation_residuals().push_back(0.0);
    sol.rotation_residuals().push_back(0.0);
    if ((int)sol.configurations().size() >= n_desired)
      break;
  }
  return sol;
}

Eigen::Affine3d MyIkSolver::getFK(const ik_solver::Configuration& q)
{
  Eigen::Affine3d T_base_flange = Eigen::Affine3d::Identity();
  // ... forward kinematics from base_frame_ to flange_frame_ ...
  return T_base_flange;
}
}  // namespace my_ik_solver

PLUGINLIB_EXPORT_CLASS(my_ik_solver::MyIkSolver, ik_solver::IkSolver)
```

Rules:

- `getIk()` receives the **flange** pose in the **base** frame; the server already handles the tool and
  the reference frame of the request.
- Return joint values in the order of `joint_names_`.
- The node creates several instances of the plugin and calls them from different threads: do not use
  global state.
- Fill the residual vectors with one value per configuration, or leave them empty.

### 3.3 Plugin description and build

`plugin.xml`:

```xml
<library path="my_ik_solver">
  <class name="my_ik_solver/MyIkSolver" type="my_ik_solver::MyIkSolver" base_class_type="ik_solver::IkSolver">
    <description>My IK solver</description>
  </class>
</library>
```

`CMakeLists.txt` (relevant part):

```cmake
find_package(ament_cmake REQUIRED)
find_package(ik_solver REQUIRED)
find_package(pluginlib REQUIRED)

add_library(my_ik_solver SHARED src/my_ik_solver.cpp)
target_include_directories(my_ik_solver PUBLIC include)
ament_target_dependencies(my_ik_solver ik_solver pluginlib)

pluginlib_export_plugin_description_file(ik_solver plugin.xml)
install(TARGETS my_ik_solver LIBRARY DESTINATION lib)
ament_package()
```

The `ROS_X` and `_MAX_NUM_PARALLEL_IK_SOLVER` definitions needed by the `ik_solver` headers come from the
`ik_solver` target, so the plugin is compiled with the same values as the node.

Then use `type: my_ik_solver/MyIkSolver` in the configuration file.

`ik_solver_test` contains a minimal plugin (`TestIkSolver`) that you can use as a starting point, and
`ur_ik_solver` is a complete analytic solver.

## API documentation

```bash
cd ik_solver   # repository folder
doxygen Doxyfile
xdg-open doxygen/html/index.html
```
