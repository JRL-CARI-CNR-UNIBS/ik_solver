# ik_solver API reference

This page is generated from the Doxygen comments in the source code. To browse the same
documentation as HTML, run `doxygen Doxyfile` in the repository folder and open
`doxygen/html/index.html`.

It covers the C++ classes and functions and the scripts. The ROS interface of the nodes
(actions, services, topics and parameters) is described in the [README](../README.md).

## Contents

- [`ik_solver/include/ik_solver/ik_solver.hpp`](#ik_solverincludeik_solverik_solverhpp)
  - [`ik_solver::IkSolver`](#class-ik_solveriksolver)
- [`ik_solver/include/ik_solver/internal/ik_solver_node.ros2.hpp`](#ik_solverincludeik_solverinternalik_solver_noderos2hpp)
  - [`ik_solver::IkSolverNode`](#class-ik_solveriksolvernode)
- [`ik_solver/include/ik_solver/internal/services.ros2.h`](#ik_solverincludeik_solverinternalservicesros2h)
  - [`ik_solver::IkServices`](#class-ik_solverikservices)
- [`ik_solver/include/ik_solver/internal/services_common.h`](#ik_solverincludeik_solverinternalservices_commonh)
  - [`ik_solver::IkServicesBase`](#class-ik_solverikservicesbase)
- [`ik_solver/include/ik_solver/internal/types.h`](#ik_solverincludeik_solverinternaltypesh)
- [`ik_solver/include/ik_solver/internal/utils.h`](#ik_solverincludeik_solverinternalutilsh)
- [`ik_solver_core/include/ik_solver_core/ik_solver_base_class.h`](#ik_solver_coreincludeik_solver_coreik_solver_base_classh)
  - [`ik_solver::IkSolverBase`](#class-ik_solveriksolverbase)
- [`ik_solver_core/include/ik_solver_core/internal/SafeQueue.h`](#ik_solver_coreincludeik_solver_coreinternalsafequeueh)
  - [`ik_solver::SafeQueue`](#class-ik_solversafequeue)
- [`ik_solver_core/include/ik_solver_core/internal/ik_solver_base_class_impl.h`](#ik_solver_coreincludeik_solver_coreinternalik_solver_base_class_implh)
- [`ik_solver_core/include/ik_solver_core/internal/types.h`](#ik_solver_coreincludeik_solver_coreinternaltypesh)
  - [`ik_solver::Solutions`](#struct-ik_solversolutions)
- [`ik_solver_core/include/ik_solver_core/internal/utils.h`](#ik_solver_coreincludeik_solver_coreinternalutilsh)
- [`ik_solver_core/include/ik_solver_core/internal/wsq.h`](#ik_solver_coreincludeik_solver_coreinternalwsqh)
  - [`ik_solver::WorkStealingQueue`](#class-ik_solverworkstealingqueue)
  - [`ik_solver::WorkStealingQueue::Array`](#struct-ik_solverworkstealingqueuearray)
- [`ik_solver_test/include/ik_solver_test/test_ik_solver_plugin.hpp`](#ik_solver_testincludeik_solver_testtest_ik_solver_pluginhpp)
  - [`test_ik_solver::TestIkSolver`](#class-test_ik_solvertestiksolver)
- [Scripts](#scripts)

## ik_solver/include/ik_solver/ik_solver.hpp

ROS base class of the IK solver plugins.

### class `ik_solver::IkSolver`

Inherits from `ik_solver::IkSolverBase`.

Base class of the IK solver plugins, loaded with pluginlib.

Adds to `IkSolverBase` the TF lookup through a tf2 buffer. A plugin derives from this class, implements `config()`, `getIk()` and `getFK()`, and is exported with `PLUGINLIB_EXPORT_CLASS(my_ns::MySolver, ik_solver::IkSolver`).

#### Public functions

##### `IkSolver`

```cpp
IkSolver()
```

##### `~IkSolver`

```cpp
virtual ~IkSolver()=default
```

##### `setBuffer`

```cpp
void setBuffer(const tf2_ros::Buffer::SharedPtr &buffer)
```

Set the TF buffer.

It must be called before `config()`.

##### `getTF`

```cpp
virtual bool getTF(const std::string &a_name, const std::string &b_name, Eigen::Affine3d &T_ab) const
```

Get the pose of `b_name` in `a_name` from TF (up to 50 attempts of 5 s).

**Parameters:**

- `a_name`: Reference frame.
- `b_name`: Target frame.
- `T_ab` [out]: Pose of b in a.

**Returns:** False if the transform is not available or the buffer is not set.

#### Protected attributes

| Member | Type | Description |
|---|---|---|
| `tf_buffer_` | `tf2_ros::Buffer::SharedPtr` | TF buffer used by `getTF()`. |

## ik_solver/include/ik_solver/internal/ik_solver_node.ros2.hpp

ROS 2 node that loads an IK solver plugin and offers the IK/FK services.

### class `ik_solver::IkSolverNode`

Inherits from `rclcpp::Node`.

ROS 2 node that loads an IK solver plugin and offers the IK/FK services.

The plugin name is read from the parameter &lt;namespace&gt;/type. The URDF is read from the ROS parameter `robot_description` or, if missing, from the topic given by &lt;namespace&gt;/robot_description_topic (default `~/robot_description`). Then MAX_NUM_PARALLEL_IK_SOLVER instances of the plugin are created and configured.

Usage:

```cpp
auto node = ik_solver::IkSolverNode::make_node("ik_solver_node");
while (!node->ready())
  rclcpp::spin_some(node->get_node_base_interface());
rclcpp::executors::MultiThreadedExecutor executor;
node->setup_services_and_run(executor);
```

#### Public functions

##### `ready`

```cpp
bool ready()
```

True when the solvers are created and configured.

##### `setup_services_and_run`

```cpp
void setup_services_and_run(rclcpp::Executor &ex)
```

Advertise the services and spin the executor (blocking).

**Parameters:**

- `ex`: Executor; a multi-threaded executor allows concurrent requests.

#### Public static functions

##### `make_node`

```cpp
static std::shared_ptr< IkSolverNode > make_node(const std::string &name, const rclcpp::NodeOptions &opt=rclcpp::NodeOptions())
```

Create the node.

**Parameters:**

- `name`: Node name.
- `opt`: Node options.

**Returns:** The node.

#### Protected functions

##### `configure_after_robot_description`

```cpp
void configure_after_robot_description(const std_msgs::msg::String &msg)
```

Store the URDF, then create and configure the solver pool.

## ik_solver/include/ik_solver/internal/services.ros2.h

ROS 2 services of the IK solver node.

### class `ik_solver::IkServices`

Inherits from `ik_solver::IkServicesBase`.

ROS 2 services of the IK solver node.

Advertised services (relative to the node namespace): `get_ik`, `get_ik_array`, `get_task_redundant_ik_array`, `get_fk`, `get_fk_array`, `get_bounds`, `set_bounds`, `get_frames`, `reconfigure`, `change_tool`, `set_initial_configuration`. The services exist as long as this object exists.

#### Public functions

##### `~IkServices`

```cpp
~IkServices()=default
```

##### `IkServices`

```cpp
IkServices(rclcpp::Node::SharedPtr &nh, IkSolversPool &ik_solvers)
```

Create the services.

Construct a new Ik Services:: Ik Services object.

**Parameters:**

- `nh`: Node.
- `ik_solvers`: Configured solver pool.
- `nh`: 
- `ik_solvers`: 

##### `computeIK`

```cpp
bool computeIK(const ik_solver_msgs::srv::GetIk::Request::SharedPtr req, ik_solver_msgs::srv::GetIk::Response::SharedPtr res)
```

GetIk service: IK of one tool pose.

**Parameters:**

- `req`: 
- `res`: 

**Returns:** true

**Returns:** false

##### `computeFK`

```cpp
bool computeFK(const ik_solver_msgs::srv::GetFk::Request::SharedPtr req, ik_solver_msgs::srv::GetFk::Response::SharedPtr res)
```

GetFk service: pose of the tip frame for one configuration.

##### `computeIKArray`

```cpp
bool computeIKArray(const ik_solver_msgs::srv::GetIkArray::Request::SharedPtr req, ik_solver_msgs::srv::GetIkArray::Response::SharedPtr res)
```

GetIkArray service: IK of many tool poses, sequential or parallel according to `parallelize`.

**Parameters:**

- `req`: 
- `res`: 

**Returns:** true

**Returns:** false

##### `computeTaskRedundantIKArray`

```cpp
bool computeTaskRedundantIKArray(const ik_solver_msgs::srv::GetIkArray::Request::SharedPtr req, ik_solver_msgs::srv::GetIkArray::Response::SharedPtr res)
```

Task-redundant IK: IK of each target pose perturbed by all the perturbations of `computeTaskRedundantPerturbations()`.

The solutions of all perturbations of a target are merged and, if `filter_duplicates` is set, duplicated configurations are removed.

##### `computeFKArray`

```cpp
bool computeFKArray(const ik_solver_msgs::srv::GetFkArray::Request::SharedPtr req, ik_solver_msgs::srv::GetFkArray::Response::SharedPtr res)
```

GetFkArray service: poses of the tip frame for many configurations (parallel).

##### `getBounds`

```cpp
bool getBounds(const ik_solver_msgs::srv::GetBound::Request::SharedPtr req, ik_solver_msgs::srv::GetBound::Response::SharedPtr res)
```

GetBound service: joint limits.

**Parameters:**

- `req`: 
- `res`: 

**Returns:** true

**Returns:** false

##### `setBounds`

```cpp
bool setBounds(const ik_solver_msgs::srv::SetBound::Request::SharedPtr req, ik_solver_msgs::srv::SetBound::Response::SharedPtr res)
```

SetBound service: change the joint limits of all solvers (see ik_solver_msgs/SetBound).

##### `getFrames`

```cpp
bool getFrames(const ik_solver_msgs::GetFrames::Request::SharedPtr req, ik_solver_msgs::GetFrames::Response::SharedPtr res)
```

GetFrames service: base, flange and tool frames.

**Parameters:**

- `req`: 
- `res`: 

**Returns:** true

**Returns:** false

##### `reconfigure`

```cpp
bool reconfigure(const Trigger::Request::SharedPtr req, Trigger::Response::SharedPtr res)
```

Reconfigure service: reload the parameters of all solvers.

**Parameters:**

- `req`: 
- `res`: 

**Returns:** true

**Returns:** false

##### `changeTool`

```cpp
void changeTool(ik_solver_msgs::srv::ChangeTool::Request::SharedPtr req, ik_solver_msgs::srv::ChangeTool::Response::SharedPtr res)
```

ChangeTool service: change the tool frame of all solvers.

##### `setInitialConfiguration`

```cpp
bool setInitialConfiguration(const ik_solver_msgs::srv::SetInitialConfiguration::Request::SharedPtr req, ik_solver_msgs::srv::SetInitialConfiguration::Response::SharedPtr res)
```

SetInitialConfiguration service.

## ik_solver/include/ik_solver/internal/services_common.h

ROS-version-independent implementation of the IK/FK services.

### class `ik_solver::IkServicesBase`

Implementation of the IK/FK services, shared by the ROS 1 and ROS 2 nodes.

Service requests express target poses as the pose of the tool (or tip) frame in the frame given by the pose header; the services convert them to flange poses in the base frame before calling the solvers.

Parameters read under the solver namespace (besides those of `IkSolverBase`):

- `update_recursively_seeds`: default seed update mode of IK arrays;
- `task_redundant/`...: perturbations used by the task-redundant IK (see README);
- `filter_duplicates` (bool, default true): remove duplicated solutions in the task-redundant IK;
- `initial_conf` (list of double): reference configuration used to choose among duplicates.

#### Public functions

##### `loadTaskRedundantParams`

```cpp
void loadTaskRedundantParams()
```

Read `filter_duplicates` and `initial_conf`.

##### `setInitialConfiguration`

```cpp
void setInitialConfiguration(const std::vector< double > &conf)
```

Set the reference configuration used to choose among duplicates.

##### `getInitialConfiguration`

```cpp
const std::vector< double > & getInitialConfiguration() const
```

Reference configuration.

##### `setFilterDuplicates`

```cpp
void setFilterDuplicates(bool filter)
```

Enable or disable duplicate filtering.

##### `getFilterDuplicates`

```cpp
bool getFilterDuplicates() const
```

True if duplicate filtering is enabled.

##### `setInitialConfiguration`

```cpp
bool setInitialConfiguration(ik_solver_msgs::SetInitialConfiguration::Request *req, ik_solver_msgs::SetInitialConfiguration::Response *res)
```

SetInitialConfiguration service.

#### Protected functions

##### `config`

```cpp
const IkSolver & config() const
```

First solver of the pool, used to read the configuration.

##### `IkServicesBase`

```cpp
IkServicesBase(IkSolversPool &ik_solvers)
```

Constructor.

**Parameters:**

- `ik_solvers`: Configured solver pool.

##### `computeTransformations`

```cpp
bool computeTransformations(const std::string &tip_frame, const std::string &reference_frame, Eigen::Affine3d &T_poses_base, Eigen::Affine3d &T_flange_tool, Eigen::Affine3d &T_tool_tip)
```

Compute the transforms needed by the FK services.

**Parameters:**

- `tip_frame`: Frame whose pose is requested (empty: tool frame).
- `reference_frame`: Frame of the returned poses.
- `T_poses_base` [out]: Pose of the base in the reference frame.
- `T_flange_tool` [out]: Pose of the tool in the flange frame.
- `T_tool_tip` [out]: Pose of the tip in the tool frame.

**Returns:** False if a transform is missing.

##### `computeIKArrayMT`

```cpp
std::vector< ik_solver::Solutions > computeIKArrayMT(const std::vector< Eigen::Affine3d > &v_T_b_f, const std::vector< ik_solver::Configurations > &vseeds, size_t desired_solutions, int min_stall_iterations, int max_stall_iterations, size_t num_threads=0)
```

Solve many IK problems in parallel, one per worker thread.

**Parameters:**

- `v_T_b_f`: Flange poses in the base frame.
- `vseeds`: Seeds of each pose.
- `desired_solutions`: Number of solutions per pose.
- `min_stall_iterations`: Minimum stall iterations.
- `max_stall_iterations`: Maximum stall iterations.
- `num_threads`: Number of worker threads; 0 or more than MAX_NUM_PARALLEL_IK_SOLVER means MAX_NUM_PARALLEL_IK_SOLVER. Never more threads than poses are started.

**Returns:** `Solutions` of each pose.

##### `computeIKArrayST`

```cpp
std::vector< ik_solver::Solutions > computeIKArrayST(const std::vector< Eigen::Affine3d > &v_T_b_f, const std::vector< ik_solver::Configurations > &vseeds, size_t desired_solutions, int min_stall_iterations, int max_stall_iterations, uint8_t update_recursively_seeds)
```

Solve many IK problems sequentially.

The solutions of each pose are added as seeds of the next one.

**Parameters:**

- `v_T_b_f`: Flange poses in the base frame.
- `vseeds`: Seeds of each pose.
- `desired_solutions`: Number of solutions per pose.
- `min_stall_iterations`: Minimum stall iterations.
- `max_stall_iterations`: Maximum stall iterations.
- `update_recursively_seeds`: Seed update mode.

**Returns:** `Solutions` of each pose.

##### `computeIK`

```cpp
bool computeIK(ik_solver_msgs::GetIk::Request *req, ik_solver_msgs::GetIk::Response *res)
```

GetIk service: IK of one tool pose.

**Parameters:**

- `req`: 
- `res`: 

**Returns:** true

**Returns:** false

##### `computeFK`

```cpp
bool computeFK(ik_solver_msgs::GetFk::Request *req, ik_solver_msgs::GetFk::Response *res)
```

GetFk service: pose of the tip frame for one configuration.

##### `computeIKArray`

```cpp
bool computeIKArray(ik_solver_msgs::GetIkArray::Request *req, ik_solver_msgs::GetIkArray::Response *res)
```

GetIkArray service: IK of many tool poses, sequential or parallel according to `parallelize`.

**Parameters:**

- `req`: 
- `res`: 

**Returns:** true

**Returns:** false

##### `computeTaskRedundantIKArray`

```cpp
bool computeTaskRedundantIKArray(ik_solver_msgs::GetIkArray::Request *req, ik_solver_msgs::GetIkArray::Response *res)
```

Task-redundant IK: IK of each target pose perturbed by all the perturbations of `computeTaskRedundantPerturbations()`.

The solutions of all perturbations of a target are merged and, if `filter_duplicates` is set, duplicated configurations are removed.

##### `computeTaskRedundantPerturbations`

```cpp
std::vector< Eigen::Affine3d > computeTaskRedundantPerturbations() const
```

Build the pose perturbations from the `task_redundant` parameters (grid or random).

Reads the 'task_redundant' parameters (namespace under the solver's param namespace) and builds the set of pose perturbations to be applied to each requested target.

Two mutually exclusive modes are supported, selected by the string parameter 'task_redundant/type':

- "grid": for each axis (x,y,z,roll,pitch,yaw) reads min/max/step and builds the cartesian product.
- "random": for each axis reads mean/standard_deviation and draws 'task_redundant/n_samples' samples, optionally seeded deterministically via 'task_redundant/seed' (falls back to a non-deterministic seed when unset). An axis that is not configured defaults to a single perturbation value of 0 on that axis.

##### `computeFKArray`

```cpp
bool computeFKArray(ik_solver_msgs::GetFkArray::Request *req, ik_solver_msgs::GetFkArray::Response *res)
```

GetFkArray service: poses of the tip frame for many configurations (parallel).

##### `getBounds`

```cpp
bool getBounds(ik_solver_msgs::GetBound::Request *req, ik_solver_msgs::GetBound::Response *res)
```

GetBound service: joint limits.

**Parameters:**

- `req`: 
- `res`: 

**Returns:** true

**Returns:** false

##### `setBounds`

```cpp
bool setBounds(ik_solver_msgs::SetBound::Request *req, ik_solver_msgs::SetBound::Response *res)
```

SetBound service: change the joint limits of all solvers (see ik_solver_msgs/SetBound).

##### `getFrames`

```cpp
bool getFrames(ik_solver_msgs::srv::GetFrames::Request *req, ik_solver_msgs::srv::GetFrames::Response *res)
```

GetFrames service: base, flange and tool frames.

**Parameters:**

- `req`: 
- `res`: 

**Returns:** true

**Returns:** false

##### `reconfigure`

```cpp
bool reconfigure(Trigger::Request *req, Trigger::Response *res)
```

Reconfigure service: reload the parameters of all solvers.

**Parameters:**

- `req`: 
- `res`: 

**Returns:** true

**Returns:** false

##### `changeTool`

```cpp
void changeTool(ik_solver_msgs::ChangeTool::Request *req, ik_solver_msgs::ChangeTool::Response *res)
```

ChangeTool service: change the tool frame of all solvers.

#### Protected attributes

| Member | Type | Description |
|---|---|---|
| `ik_solvers_` | `IkSolversPool &` | Solver pool. |
| `filter_duplicates_` | `bool` | Remove duplicated solutions in the task-redundant IK. |
| `initial_conf_` | `std::vector< double >` | Reference configuration used to choose among duplicates. |
| `initial_conf_set_manually_` | `bool` | True if initial_conf_ was set by service or setter (parameters do not override it). |

### Free functions and types

#### `IkArgs`

```cpp
using ik_solver::IkArgs = std::tuple<Eigen::Affine3d, ik_solver::Configurations, std::size_t, int, int>
```

Arguments of one IK job: (T_base_flange, seeds, desired solutions, min stall iterations, max stall iterations).

#### `get_T_base_flange`

```cpp
const Eigen::Affine3d & get_T_base_flange(const IkArgs &args)
```

Flange pose of an IK job.

#### `get_seeds`

```cpp
const ik_solver::Configurations & get_seeds(const IkArgs &args)
```

Seeds of an IK job.

#### `get_desired_solutions`

```cpp
const std::size_t & get_desired_solutions(const IkArgs &args)
```

Desired solutions of an IK job.

#### `get_min_stall_iterations`

```cpp
const int & get_min_stall_iterations(const IkArgs &args)
```

Minimum stall iterations of an IK job.

#### `get_max_stall_iterations`

```cpp
const int & get_max_stall_iterations(const IkArgs &args)
```

Maximum stall iterations of an IK job.

#### `MAX_NUM_PARALLEL_IK_SOLVER`

```cpp
static constexpr const size_t MAX_NUM_PARALLEL_IK_SOLVER =
```

Number of solver instances (and worker threads) of a node.

Set at build time by CMake through _MAX_NUM_PARALLEL_IK_SOLVER: number of CPU cores minus 2 (10 on machines with 3 cores or less).

#### `IkSolversPool`

```cpp
using ik_solver::IkSolversPool = std::array<std::shared_ptr<ik_solver::IkSolver>, ik_solver::MAX_NUM_PARALLEL_IK_SOLVER>
```

Pool of solver instances, one per worker thread.

## ik_solver/include/ik_solver/internal/types.h

Conversions between `ik_solver` types and `ik_solver_msgs` messages.

### Free functions and types

#### `Trigger`

```cpp
using Trigger = std_srvs::srv::Trigger
```

#### `Pose`

```cpp
using Pose = geometry_msgs::msg::Pose
```

#### `cast`

```cpp
ik_solver_msgs::Configuration & cast(ik_solver_msgs::Configuration &lhs, const ik_solver::Configuration &rhs)
```

Copy a configuration into a message.

#### `cast`

```cpp
ik_solver_msgs::Configuration cast(const ik_solver::Configuration &rhs)
```

Convert a configuration to a message.

#### `operator<<`

```cpp
ik_solver_msgs::Configuration & operator<<(ik_solver_msgs::Configuration &lhs, const ik_solver::Configuration &rhs)
```

#### `cast`

```cpp
ik_solver::Configuration & cast(ik_solver::Configuration &lhs, const ik_solver_msgs::Configuration &rhs)
```

Copy a message into a configuration.

#### `cast`

```cpp
ik_solver::Configuration cast(const ik_solver_msgs::Configuration &rhs)
```

Convert a message to a configuration.

#### `operator<<`

```cpp
ik_solver::Configuration & operator<<(ik_solver::Configuration &lhs, const ik_solver_msgs::Configuration &rhs)
```

#### `cast`

```cpp
ik_solver_msgs::IkSolution & cast(ik_solver_msgs::IkSolution &lhs, const ik_solver::Solutions &rhs)
```

Copy solutions into an IkSolution message.

#### `cast`

```cpp
ik_solver_msgs::IkSolution cast(const ik_solver::Solutions &rhs)
```

Convert solutions to an IkSolution message.

#### `cast`

```cpp
ik_solver::Solutions & cast(ik_solver::Solutions &lhs, const ik_solver_msgs::IkSolution &rhs)
```

Copy an IkSolution message into solutions.

#### `cast`

```cpp
ik_solver::Solutions cast(const ik_solver_msgs::IkSolution &rhs)
```

Convert an IkSolution message to solutions.

#### `operator<<`

```cpp
ik_solver::Solutions & operator<<(ik_solver::Solutions &lhs, const ik_solver_msgs::IkSolution &rhs)
```

#### `cast`

```cpp
std::vector< ik_solver_msgs::IkSolution > & cast(std::vector< ik_solver_msgs::IkSolution > &lhs, const std::vector< ik_solver::Solutions > &rhs)
```

#### `cast`

```cpp
std::vector< ik_solver_msgs::IkSolution > cast(const std::vector< ik_solver::Solutions > &rhs)
```

Convert a list of solutions to messages.

#### `operator<<`

```cpp
std::vector< ik_solver_msgs::IkSolution > & operator<<(std::vector< ik_solver_msgs::IkSolution > &lhs, const std::vector< ik_solver::Solutions > &rhs)
```

## ik_solver/include/ik_solver/internal/utils.h

ROS utilities of the IK solver node.

### Free functions and types

#### `getSeeds`

```cpp
Configurations getSeeds(const std::vector< std::string > &joint_names, const std::vector< std::string > &seed_names, const std::vector< ik_solver_msgs::Configuration > &seeds)
```

Convert seed messages to configurations in solver joint order.

**Parameters:**

- `joint_names`: Joint names of the solver.
- `seed_names`: Joint names of the seeds.
- `seeds`: Seed messages.

**Returns:** Seeds in solver joint order.

#### `to_string`

```cpp
std::string to_string(const Pose &mat)
```

String representation of a pose message.

#### `from_pose_to_eigen`

```cpp
void from_pose_to_eigen(const Pose &pose, Eigen::Affine3d &eig)
```

Convert a pose message to an Eigen transform.

#### `from_eigen_to_pose`

```cpp
void from_eigen_to_pose(const Eigen::Affine3d &eig, Pose &pose)
```

Convert an Eigen transform to a pose message.

## ik_solver_core/include/ik_solver_core/ik_solver_base_class.h

ROS-independent base class of the inverse kinematics solvers.

### class `ik_solver::IkSolverBase`

ROS-independent base class of an inverse kinematics (IK) solver.

A concrete solver implements `getIk()` and `getFK()` for a chain from `base_frame` to `flange_frame`; the base class handles parameters, joint limits, the tool frame (`flange_frame` → `tool_frame` transform) and the conversions between frames. `getTF()` is implemented by the ROS layer (`ik_solver::IkSolver`).

Frame naming: `T_a_b` (or `T_ab`) is the pose of frame b expressed in frame a.

Parameters read by `config()` under the namespace `param_ns`:

- `base_frame`, `flange_frame`, `tool_frame` (string, required);
- `joint_names` (list of strings, required): joints of the chain, in solver order;
- `robot_description` (string, required): URDF, used to read the joint limits;
- `limits/` &lt;joint&gt;/upper, `limits/` &lt;joint&gt;/lower (double, optional): restrict the URDF limits (a value wider than the URDF one is ignored); they can be changed at runtime with `setBounds()`;
- `desired_solutions` (int, default 8): number of solutions to look for;
- `min_stall_iterations` (int, default 998), `max_stall_iterations` (int, default 999): iterations without new solutions before stopping (meaning depends on the solver);
- `parallel_ik_mode` (int, default 2 = disabled): default parallelization mode of IK arrays.

The environment variable `IK_SOLVER_LOGGER_CONFIG_PATH` must point to a cnr_logger configuration file.

#### Public functions

##### `IkSolverBase`

```cpp
IkSolverBase()=default
```

##### `~IkSolverBase`

```cpp
virtual ~IkSolverBase()=default
```

##### `config`

```cpp
virtual bool config(const std::string &param_ns="")
```

Read the parameters and the joint limits (see class description).

Derived classes overriding this function should call it first.

**Parameters:**

- `param_ns`: Parameter namespace.

**Returns:** False if a required parameter is missing, a joint is not in the URDF, or a transform is missing.

##### `getIk`

```cpp
virtual Solutions getIk(const Eigen::Affine3d &T_base_flange, const Configurations &seeds, const int &desired_solutions=-1, const int &min_stall_iterations=-1, const int &max_stall_iterations=-1)=0
```

Compute the IK solutions of a flange pose.

Implemented by each solver.

**Parameters:**

- `T_base_flange`: Pose of the flange in the base frame.
- `seeds`: Initial guesses (may be empty).
- `desired_solutions`: Number of solutions to look for (-1: use the parameter).
- `min_stall_iterations`: Minimum iterations without new solutions (-1: use the parameter).
- `max_stall_iterations`: Maximum iterations without new solutions (-1: use the parameter).

**Returns:** The solutions found (possibly none).

##### `getFK`

```cpp
virtual Eigen::Affine3d getFK(const Configuration &s)=0
```

Compute the forward kinematics.

Implemented by each solver.

**Parameters:**

- `s`: Joint configuration, in the order of `joint_names()`.

**Returns:** Pose of the flange in the base frame.

##### `computeIk`

```cpp
Solutions computeIk(const Eigen::Affine3d &T_world_tool, const Eigen::Affine3d &T_base_world, const Eigen::Affine3d &T_tool_flange, const Configurations &seeds, const int &desired_solutions=-1, const int &min_stall_iterations=-1, const int &max_stall_iterations=-1)
```

Compute the IK of a tool pose, given the frame transforms.

**Parameters:**

- `T_world_tool`: Pose of the tool in the world frame.
- `T_base_world`: Pose of the world in the base frame: the flange pose is T_base_world * T_world_tool * T_tool_flange.
- `T_tool_flange`: Pose of the flange in the tool frame.
- `seeds`: Initial guesses.
- `desired_solutions`: Number of solutions (-1: use the parameter).
- `min_stall_iterations`: Minimum stall iterations (-1: use the parameter).
- `max_stall_iterations`: Maximum stall iterations (-1: use the parameter).

**Returns:** The solutions.

##### `computeFk`

```cpp
Eigen::Affine3d computeFk(const Configuration &s, const Eigen::Affine3d &T_world_base, const Eigen::Affine3d &T_flange_tool)
```

Compute the tool pose in the world frame, given the frame transforms.

**Parameters:**

- `s`: Joint configuration.
- `T_world_base`: Pose of the base in the world frame.
- `T_flange_tool`: Pose of the tool in the flange frame.

**Returns:** Pose of the tool in the world frame.

##### `computeIk`

```cpp
Solutions computeIk(const Eigen::Affine3d &T_world_tool, const std::string &world_frame, const std::string &tool_frame, const Configurations &seeds, const int &desired_solutions=-1, const int &min_stall_iterations=-1, const int &max_stall_iterations=-1)
```

Compute the IK of a tool pose; the transforms are read with `getTF()`.

**Parameters:**

- `T_world_tool`: Pose of the tool in `world_frame`.
- `world_frame`: Frame of the pose.
- `tool_frame`: Tool frame.
- `seeds`: Initial guesses.
- `desired_solutions`: Number of solutions (-1: use the parameter).
- `min_stall_iterations`: Minimum stall iterations (-1: use the parameter).
- `max_stall_iterations`: Maximum stall iterations (-1: use the parameter).

**Returns:** The solutions (empty if a transform is missing).

##### `computeFk`

```cpp
Eigen::Affine3d computeFk(const Configuration &s, const std::string &world_frame, const std::string &base_frame)
```

Compute the pose of the current tool in `world_frame`; the transforms are read with `getTF()`.

**Parameters:**

- `s`: Joint configuration.
- `world_frame`: Reference frame of the result.
- `base_frame`: Unused: the configured base frame is used.

**Returns:** Pose of the tool (uninitialized if a transform is missing).

##### `joint_names`

```cpp
const std::vector< std::string > & joint_names() const
```

Joint names, in solver order.

##### `base_frame`

```cpp
const std::string & base_frame() const
```

Base frame of the kinematic chain.

##### `flange_frame`

```cpp
const std::string & flange_frame() const
```

Flange (last link) frame of the kinematic chain.

##### `tool_frame`

```cpp
const std::string & tool_frame() const
```

Current tool frame.

##### `transform_from_flange_to_tool`

```cpp
const Eigen::Affine3d & transform_from_flange_to_tool() const
```

Pose of the flange in the tool frame (T_tool_flange).

##### `transform_from_tool_to_flange`

```cpp
Eigen::Affine3d transform_from_tool_to_flange() const
```

Pose of the tool in the flange frame (inverse of `transform_from_flange_to_tool()`).

##### `lb`

```cpp
const Configuration & lb() const
```

Lower joint limits.

##### `ub`

```cpp
const Configuration & ub() const
```

Upper joint limits.

##### `setBounds`

```cpp
bool setBounds(const Configuration &lb, const Configuration &ub)
```

Set the joint limits, clamped to the URDF limits.

The limits are restored from the parameters by `config()`.

**Parameters:**

- `lb`: Lower limits (one per joint).
- `ub`: Upper limits (one per joint).

**Returns:** False (limits unchanged) if the sizes are wrong or, after clamping, a lower limit is greater than the upper one.

##### `revolute`

```cpp
const std::vector< bool > & revolute() const
```

True for revolute joints.

##### `min_stall_iterations`

```cpp
const int & min_stall_iterations() const
```

Default minimum stall iterations.

##### `max_stall_iterations`

```cpp
const int & max_stall_iterations() const
```

Default maximum stall iterations.

##### `desired_solutions`

```cpp
const int & desired_solutions() const
```

Default number of desired solutions.

##### `parallelize`

```cpp
const int & parallelize() const
```

Default parallelization mode of IK arrays (values of ik_solver_msgs/GetIkArray).

##### `param_namespace`

```cpp
const std::string param_namespace() const
```

Parameter namespace (always ending with '/').

##### `changeTool`

```cpp
bool changeTool(const std::string &t_frame)
```

Change the tool frame, reading the flange-tool transform with `getTF()`.

**Parameters:**

- `t_frame`: New tool frame.

**Returns:** False if the transform is not available.

##### `changeTool`

```cpp
bool changeTool(const std::string &t_frame, const Eigen::Affine3d &T_tool_flange)
```

Change the tool frame using a given transform (not checked).

**Parameters:**

- `t_frame`: New tool frame.
- `T_tool_flange`: Pose of the flange in the new tool frame.

**Returns:** Always true.

##### `getTF`

```cpp
virtual bool getTF(const std::string &a_name, const std::string &b_name, Eigen::Affine3d &T_ab) const =0
```

Get the pose of frame `b_name` in frame `a_name`.

Implemented by the ROS layer.

**Parameters:**

- `a_name`: Reference frame.
- `b_name`: Target frame.
- `T_ab` [out]: Pose of b in a.

**Returns:** False if the transform is not available.

#### Protected functions

##### `getFlangeTool`

```cpp
bool getFlangeTool()
```

Read the flange-tool transform with `getTF()`.

#### Protected attributes

| Member | Type | Description |
|---|---|---|
| `logger_` | `cnr_logger::TraceLogger` | Logger, configured from IK_SOLVER_LOGGER_CONFIG_PATH. |
| `logger_initialized_` | `bool` | True after the first successful logger_.init(). |
| `params_ns_` | `std::string` | Parameter namespace. |
| `robot_description_` | `std::string` | URDF. |
| `T_tool_flange_` | `Eigen::Affine3d` | Pose of the flange in the tool frame. |
| `base_frame_` | `std::string` | Base frame. |
| `flange_frame_` | `std::string` | Flange frame. |
| `tool_frame_` | `std::string` | Tool frame. |
| `joint_names_` | `std::vector< std::string >` | Joint names. |
| `ub_` | `Configuration` | Upper joint limits. |
| `lb_` | `Configuration` | Lower joint limits. |
| `urdf_ub_` | `Configuration` | Upper joint limits of the URDF. |
| `urdf_lb_` | `Configuration` | Lower joint limits of the URDF. |
| `revolute_` | `std::vector< bool >` | True for revolute joints. |
| `min_stall_iter_` | `int` | Default minimum stall iterations. |
| `max_stall_iter_` | `int` | Default maximum stall iterations. |
| `max_iter_` | `int` | Maximum iterations (available to solvers). |
| `desired_solutions_` | `int` | Default number of desired solutions. |
| `parallelize_` | `int` | Default parallelization mode. |
| `exploit_solutions_as_seed_` | `int` | Unused. |
| `model_` | `urdf::ModelInterfaceSharedPtr` | URDF model. |

#### Protected static attributes

| Member | Type | Description |
|---|---|---|
| `PARALLELIZE_DISABLE` | `constexpr int` | Value of parallel_ik_mode that disables parallelization. |

## ik_solver_core/include/ik_solver_core/internal/SafeQueue.h

Mutex-protected queue.

### class `ik_solver::SafeQueue`

```cpp
template <typename T>
class SafeQueue
```

Thread-safe FIFO queue based on std::queue and a mutex.

Used to distribute IK/FK jobs to worker threads.

#### Public functions

##### `SafeQueue`

```cpp
SafeQueue()
```

##### `SafeQueue`

```cpp
SafeQueue(SafeQueue &other)
```

##### `~SafeQueue`

```cpp
~SafeQueue()
```

##### `empty`

```cpp
bool empty()
```

True if the queue is empty.

##### `size`

```cpp
int size()
```

Number of elements.

##### `enqueue`

```cpp
void enqueue(T &t)
```

Push an element at the back.

##### `dequeue`

```cpp
bool dequeue(T &t)
```

Pop the front element.

**Parameters:**

- `t` [out]: Popped element.

**Returns:** False if the queue is empty.

## ik_solver_core/include/ik_solver_core/internal/ik_solver_base_class_impl.h

### Free functions and types

#### `ENV_LOGGER_CONFIG_PATH`

```cpp
static constexpr std::string_view ENV_LOGGER_CONFIG_PATH = "IK_SOLVER_LOGGER_CONFIG_PATH"
```

## ik_solver_core/include/ik_solver_core/internal/types.h

Basic types of the IK solvers.

### struct `ik_solver::Solutions`

Inherits from `std::tuple< ik_solver::Configurations, std::vector< double >, std::vector< double >, std::string >`.

Result of an IK query: configurations with their residual errors and a message.

The i-th residuals refer to the i-th configuration. Residual vectors may be empty if the solver does not compute them.

#### Public functions

##### `configurations`

```cpp
const ik_solver::Configurations & configurations() const
```

Joint configurations found.

##### `configurations`

```cpp
ik_solver::Configurations & configurations()
```

##### `translation_residuals`

```cpp
const std::vector< double > & translation_residuals() const
```

Translation error of each configuration.

##### `translation_residuals`

```cpp
std::vector< double > & translation_residuals()
```

##### `rotation_residuals`

```cpp
const std::vector< double > & rotation_residuals() const
```

Rotation error of each configuration.

##### `rotation_residuals`

```cpp
std::vector< double > & rotation_residuals()
```

##### `message`

```cpp
const std::string & message() const
```

Optional message from the solver.

##### `message`

```cpp
std::string & message()
```

##### `clear`

```cpp
void clear()
```

Remove all configurations, residuals and the message.

### Free functions and types

#### `Configuration`

```cpp
using ik_solver::Configuration = Eigen::VectorXd
```

A joint configuration.

#### `Configurations`

```cpp
using ik_solver::Configurations = std::vector<Configuration>
```

A list of joint configurations.

## ik_solver_core/include/ik_solver_core/internal/utils.h

Utilities of the IK solvers: string conversions, parameter helpers and joint-space helpers.

### Free functions and types

#### `to_string`

```cpp
std::string to_string(const Eigen::Affine3d &mat)
```

String representation of a transform matrix.

**Parameters:**

- `ss`: 

**Returns:** std::string

#### `to_string`

```cpp
std::string to_string(const ik_solver::Configuration &mat)
```

String representation of a configuration (row vectors only; column vectors give an empty string).

#### `to_string`

```cpp
std::string to_string(const ik_solver::Configurations &seeds)
```

Comma-separated representation of a list of configurations.

#### `printProgress`

```cpp
void printProgress(double percentage, const char *msg,...)
```

Print a progress bar on stdout (same line).

**Parameters:**

- `percentage`: Progress in [0, 1].
- `msg`: printf-like message printed after the bar.

#### `ltrim`

```cpp
std::string ltrim(const std::string &s, const std::string &what=" \n\r\t\f\v")
```

Remove the characters in `what` from the beginning of `s`.

#### `rtrim`

```cpp
std::string rtrim(const std::string &s, const std::string &what=" \n\r\t\f\v")
```

Remove the characters in `what` from the end of `s`.

#### `trim`

```cpp
std::string trim(const std::string &s, const std::string &what=" \n\r\t\f\v")
```

Remove whitespace from both ends of `s` (`what` is ignored).

#### `diff`

```cpp
template <typename T>
T diff(const T &a, const T &b)
```

Difference a - b.

Specialized for std::vector&lt;double&gt; and Eigen::VectorXd.

#### `norm`

```cpp
template <typename T>
double norm(const T &vv)
```

Norm of a vector.

Specialized for std::vector&lt;double&gt; (squared norm) and Eigen::VectorXd (Euclidean norm).

#### `is_the_same`

```cpp
template <typename T>
bool is_the_same(const T &lhs, const T &rhs)
```

Equality check, specialized for vectors of strings.

#### `check_and_set_duplicate_params`

```cpp
template <typename T>
bool check_and_set_duplicate_params(const std::string &nh_ns, const std::string &robot_ns, const std::string &param_name, const T &nh_val)
```

Check that a parameter has the same value in two namespaces, or copy it.

If `robot_ns` + `param_name` exists, it must be equal to `nh_val`; otherwise it is set to `nh_val`.

**Returns:** False if the two values differ.

#### `check_and_get`

```cpp
template <typename T>
bool check_and_get(std::map< std::string, std::tuple< T *, bool, T > > &params)
```

Read a set of parameters with cnr_param.

**Parameters:**

- `params`: Map from full parameter name to (destination, has_default, default value).

**Returns:** False if a parameter without default is missing.

#### `get_and_return`

```cpp
template <typename T>
bool get_and_return(std::map< std::string, T * > &params)
```

Read a set of required parameters.

**Parameters:**

- `params`: Map from full parameter name to destination.

**Returns:** False if a parameter is missing.

#### `get_and_default`

```cpp
template <typename T>
bool get_and_default(std::map< std::string, std::pair< T *, T > > &params)
```

Read a set of optional parameters.

**Parameters:**

- `params`: Map from full parameter name to (destination, default value).

**Returns:** Always true.

#### `isPresent`

```cpp
bool isPresent(const Configuration &q, const Configurations &qq, double tolerance=1e-4)
```

Check if a configuration is in a list.

**Parameters:**

- `q`: Configuration.
- `qq`: List.
- `tolerance`: Distance below which two configurations are the same.

**Returns:** True if `q` is in `qq`.

#### `resolve_ns`

```cpp
std::string resolve_ns(const std::string &params_ns)
```

Normalize a parameter namespace: add a leading '/' to relative or private names and a trailing '/'.

#### `is_finite`

```cpp
template <typename Derived>
bool is_finite(const Eigen::MatrixBase< Derived > &x)
```

True if all the coefficients are finite.

#### `is_nan`

```cpp
template <typename Derived>
bool is_nan(const Eigen::MatrixBase< Derived > &x)
```

True if no coefficient is NaN (note: despite the name).

#### `getMultiplicity`

```cpp
Configurations getMultiplicity(const Configurations &sol, const Configuration &ub, const Configuration &lb, const std::vector< bool > &revolute)
```

Add the multi-turn equivalents of each solution.

For each revolute joint, the values q ± 2kπ inside the joint limits are added, and all combinations are returned (without duplicates).

**Parameters:**

- `sol`: `Solutions`.
- `ub`: Upper limits.
- `lb`: Lower limits.
- `revolute`: True for revolute joints.

**Returns:** `Solutions` and their multi-turn equivalents.

#### `outOfBound`

```cpp
std::vector< int > outOfBound(const Configuration &c, const Configuration &ub, const Configuration &lb)
```

Joints outside their limits (or NaN).

**Returns:** Indices of the joints out of bounds.

#### `diff`

```cpp
template <>
std::vector< double > diff(const std::vector< double > &vv, const std::vector< double > &ww)
```

#### `diff`

```cpp
template <>
Eigen::VectorXd diff(const Eigen::VectorXd &vv, const Eigen::VectorXd &ww)
```

#### `norm`

```cpp
template <>
double norm(const std::vector< double > &vv)
```

#### `norm`

```cpp
template <>
double norm(const Eigen::VectorXd &vv)
```

#### `at`

```cpp
template <typename T>
double & at(T &vv, size_t i)
```

Element access, specialized for std::vector&lt;double&gt; and Eigen::VectorXd.

#### `at`

```cpp
template <>
double & at(std::vector< double > &vv, size_t i)
```

#### `at`

```cpp
template <>
double & at(Eigen::VectorXd &vv, size_t i)
```

#### `is_the_same`

```cpp
template <>
bool is_the_same(const std::vector< std::string > &lhs, const std::vector< std::string > &rhs)
```

## ik_solver_core/include/ik_solver_core/internal/wsq.h

### class `ik_solver::WorkStealingQueue`

```cpp
template <typename T>
class WorkStealingQueue
```

Lock-free unbounded single-producer multiple-consumer queue.

This class implements the work stealing queue described in the paper, "Correct and Efficient Work-Stealing for Weak Memory Models," available at [https://www.di.ens.fr/~zappa/readings/ppopp13.pdf](https://www.di.ens.fr/~zappa/readings/ppopp13.pdf).

Only the queue owner can perform pop and push operations, while others can steal data from the queue.

**Template parameters:**

- `T`: data type

#### Public functions

##### `WorkStealingQueue`

```cpp
WorkStealingQueue(int64_t capacity=1024)
```

constructs the queue with a given capacity

**Parameters:**

- `capacity`: the capacity of the queue (must be power of 2)

##### `~WorkStealingQueue`

```cpp
~WorkStealingQueue()
```

destructs the queue

##### `empty`

```cpp
bool empty() const noexcept
```

queries if the queue is empty at the time of this call

##### `size`

```cpp
size_t size() const noexcept
```

queries the number of items at the time of this call

##### `capacity`

```cpp
int64_t capacity() const noexcept
```

queries the capacity of the queue

##### `push`

```cpp
template <typename O>
void push(O &&item)
```

inserts an item to the queue

Only the owner thread can insert an item to the queue. The operation can trigger the queue to resize its capacity if more space is required.

**Template parameters:**

- `O`: data type

**Parameters:**

- `item`: the item to perfect-forward to the queue

##### `pop`

```cpp
std::optional< T > pop()
```

pops out an item from the queue

Only the owner thread can pop out an item from the queue. The return can be a @std_nullopt if this operation failed (empty queue).

##### `steal`

```cpp
std::optional< T > steal()
```

steals an item from the queue

Any threads can try to steal an item from the queue. The return can be a @std_nullopt if this operation failed (not necessary empty).

### struct `ik_solver::WorkStealingQueue::Array`

#### Public functions

##### `Array`

```cpp
Array(int64_t c)
```

##### `~Array`

```cpp
~Array()
```

##### `capacity`

```cpp
int64_t capacity() const noexcept
```

##### `push`

```cpp
template <typename O>
void push(int64_t i, O &&o) noexcept
```

##### `pop`

```cpp
T pop(int64_t i) noexcept
```

##### `resize`

```cpp
Array * resize(int64_t b, int64_t t)
```

#### Public attributes

| Member | Type | Description |
|---|---|---|
| `C` | `int64_t` |  |
| `M` | `int64_t` |  |
| `S` | `std::atomic< T > *` |  |

## ik_solver_test/include/ik_solver_test/test_ik_solver_plugin.hpp

Dummy IK solver plugin used to test the node and the services.

### class `test_ik_solver::TestIkSolver`

Inherits from `ik_solver::IkSolver`.

Dummy IK solver that logs its inputs.

`getIk()` always returns one configuration with all joints at 1.0, and `getFK()` returns the identity. It is also a minimal example of a solver plugin.

#### Public functions

##### `TestIkSolver`

```cpp
TestIkSolver()
```

##### `config`

```cpp
virtual bool config(const std::string &param_ns="") override
```

Read the parameters and the joint limits (see class description).

Derived classes overriding this function should call it first.

**Parameters:**

- `param_ns`: Parameter namespace.

**Returns:** False if a required parameter is missing, a joint is not in the URDF, or a transform is missing.

##### `getIk`

```cpp
virtual Solutions getIk(const Eigen::Affine3d &T_base_flange, const Configurations &seeds, const int &desired_solutions=-1, const int &min_stall_iterations=-1, const int &max_stall_iterations=-1) override
```

Compute the IK solutions of a flange pose.

Implemented by each solver.

**Parameters:**

- `T_base_flange`: Pose of the flange in the base frame.
- `seeds`: Initial guesses (may be empty).
- `desired_solutions`: Number of solutions to look for (-1: use the parameter).
- `min_stall_iterations`: Minimum iterations without new solutions (-1: use the parameter).
- `max_stall_iterations`: Maximum iterations without new solutions (-1: use the parameter).

**Returns:** The solutions found (possibly none).

##### `getFK`

```cpp
virtual Eigen::Affine3d getFK(const Configuration &s) override
```

Compute the forward kinematics.

Implemented by each solver.

**Parameters:**

- `s`: Joint configuration, in the order of `joint_names()`.

**Returns:** Pose of the flange in the base frame.

## Scripts

### `ik_solver/scripts/get_ik_collision_check.py`

```
ROS 1 example: IK of the origin of a TF frame, followed by collision checking of the solutions.

Calls ~get_ik, then /ik_collision_check/collision_check, and publishes the solutions on /ik_solution.

Usage: rosrun ik_solver get_ik_collision_check.py <namespace> <tf_name>
```

### `ik_solver/scripts/get_tf_ik.py`

```
ROS 1 example: IK of the origin of a TF frame (~get_ik).

Publishes all the solutions in turn on /ik_solution (moveit_msgs/DisplayRobotState).

Usage: rosrun ik_solver get_tf_ik.py <namespace> <tf_name>
```

### `ik_solver/scripts/get_tf_ik_array.py`

```
ROS 1 example: IK of a sequence of poses around a TF frame (~get_ik_array).

The poses follow a circle with a superimposed small oscillation in frame tf_name; the
solutions are published on /ik_solution and the poses on /poses.

Usage: rosrun ik_solver get_tf_ik_array.py <namespace> <tf_name>
```

### `ik_solver/scripts/get_tf_ik_ros2.py`

```
ROS 2 example: IK of the origin of a TF frame (~get_ik).

Publishes all the solutions in turn on /ik_solution (moveit_msgs/DisplayRobotState).

Usage: ros2 run ik_solver get_tf_ik_ros2.py <namespace> <tf_name>
```

### `ik_solver/scripts/neighbourhood_ik.py`

```
ROS 1 service ~/neighbourhood_ik (ik_solver_msgs/NeighbourhoodPyramidIk).

Samples poses inside a pyramid in front of a TF frame (distance, width, resolution and
orientation given in the request), solves their IK with /get_ik_array, and returns the
solutions together with the unreachable poses.
```

### `ik_solver/scripts/visualizer_script.py`

```
IK Visualizer Script for RViz.

Supports both:
- Task-Redundant IK solver (get_task_redundant_ik_array / ik_task_redundant)
- Base IK solver (get_ik)

This script samples a random joint configuration, calls the selected IK service,
and visualizes the resulting configurations in RViz.

Pressing SPACE on the keyboard steps through the solutions and, once exhausted,
automatically requests a new random configuration. You can also press 'n' to
sample a new random target configuration immediately, or 'm' / 't' to toggle
between Base IK and Task-Redundant IK solvers at runtime.
```

### `ik_solver/scripts/viusalizer_script.py`

```
IK Visualizer Script for RViz.

Supports both:
- Task-Redundant IK solver (get_task_redundant_ik_array / ik_task_redundant)
- Base IK solver (get_ik)

This script samples a random joint configuration, calls the selected IK service,
and visualizes the resulting configurations in RViz.

Pressing SPACE on the keyboard steps through the solutions and, once exhausted,
automatically requests a new random configuration. You can also press 'n' to
sample a new random target configuration immediately, or 'm' / 't' to toggle
between Base IK and Task-Redundant IK solvers at runtime.
```

### `ik_solver_test/scripts/test_ik_fk.py`

```
FK -> IK round-trip test of an ik_solver node.

For many random configurations within the joint bounds, computes the pose with ~get_fk,
solves ~get_ik on that pose, and reports the IK residual errors. Solutions are published
on /ik_solution (moveit_msgs/DisplayRobotState).

Usage: ros2 run ik_solver_test test_ik_fk.py <namespace>
```

### `ik_solver_test/scripts/test_task_redundant_ik_array.py`

```
Test of the ~get_task_redundant_ik_array service of an ik_solver node.

Builds target poses from the FK of random configurations, calls the service, and logs the
number of solutions and the maximum translation/rotation residual for each target.

Usage: ros2 run ik_solver_test test_task_redundant_ik_array.py <namespace>
```

### `ik_solver_test/scripts/visualize_task_redundant_ik_array.py`

```
Show the task-redundant IK solutions of a random pose in RViz.

Computes the FK of a random configuration, calls ~get_task_redundant_ik_array on that pose,
and publishes each solution as a moveit_msgs/DisplayRobotState on /display_robot_state,
one every period_s seconds.

Usage: ros2 run ik_solver_test visualize_task_redundant_ik_array.py <namespace> [period_s]
```
