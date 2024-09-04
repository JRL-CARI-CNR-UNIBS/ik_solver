<<<<<<< HEAD
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
=======
# IK Solver

**IkSolver** is a solver-agnostic interface for inverse kinematics solvers, both numerical and analytical. It is compatible with both ROS1 and ROS2.

The solvers are provided as ROS-plugins.

The library implements parallel computation routines.

## Dependencies

A `.repo` file compatible with `vcstool` is provided with all the dependencies

- [cnr_param](https://github.com/CNR-STIIMA-IRAS/cnr_param): ROS-free parameter handling
- [cnr_logger](https://github.com/CNR-STIIMA-IRAS/cnr_logger.git): ROS-free logger library

## Usage

### Installation

#### Get dependencies
```bash
vcs import < ik_solver/ik_solver.repo
```

#### ROS 1 (catkin_tools)
```bash
## Move ros-free libraries
export INSTALL_LOCATION=/path/to/install/location
### cnr_logger
mv cnr_logger /path/outside/workspace
cd /path/outside/workspace; cd cnr_logger
mkdir build; cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$INSTALL_LOCATION -DUSE_ROS1=OFF
make; make install

### cnr_param
mv cnr_param  /path/outside/workspace
cd /path/outside/workspace; cd cnr_logger
mkdir build; cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$INSTALL_LOCATION -DUSE_ROS1=OFF
make; make install

## Build
catkin build -cs --cmake-args -DUSE_ROS1=ON
```
**Note:**
- `CMAKE_PREFIX_PATH` must contain `$INSTALL_LOCATION`;
- `LD_LIBRARY_PATH` must contain `$INSTALL_LOCATION/lib`

#### ROS 2 (colcon)
```bash
colcon build --symlink-install --continue-on-error --cmake-args -DUSE_ROS1=OFF
```

### Services

The following services are provided through the "ik_solver_node":

- `get_ik`: calls the computation of ik related to a single pose
- `get_ik_array`: calls the computation of ik related to a multiple poses
- `get_fk`: calls the computation of the forward kinematics for a single configuration
- `get_fk_array`: calls the computation of the forward kinematics for multiple configuration

Several utility services are provided:

- `get_bounds`: get joint bounds
- `change_tool`: change the tool frame

### Run server

```bash
# ROS1 : WIP
roslaunch ik_solver ik_solver_1.py plugin='[<plugin_pkg_name_1>, <plugin_pkg_name_2>, ...]' config='[<config_filename_plugin_1>, <config_filename_plugin_2>, ...]'
```

```bash
# ROS2
ros2 launch ik_solver ik_solver.launch.py file:="path/to/config/file"
```

The config file has the following structure:

```yaml
ik_solver:
  - package: "pkg1"    # Package which contain the configuration file
    config: "cf1"      # Path from package share to config file
    namespace: "n1"    # Namespace of the node (and of the config parameters)
  - package: "pkg1"
    config: "cf2"
    namespace: "n2"
```

#### Example
```bash
ros2 launch ik_solver ik_solver.launch.py file:=ik_solver/config/config_example.yaml
```

### Environment Variables
```bash
# Path to a valid directory in which save the parameters
export CNR_PARAM_ROOT_DIRECTORY="/tmp"

# Path to a valid cnr_logger config file
export IK_SOLVER_LOGGER_CONFIG_PATH="/path/to/repository/ik_solver/ik_solver/config/default_logger.yaml"
```

## Plugin configuration

Each plugin must contain a YAML file with the following configuration:

```yaml
package_name:
  type: ik_solver::CustomIkSolver # ROS Plugin Name
  base_frame: base # base frame of the chain
  flange_frame: flange # end frame of the chain

  tool_frame: tool # destination frame of the IK (it should be rigid attached to flange_frame)
  desired_solutions: 32 # number of desired solution
  joint_names:
  - joint_1
  - joint_2
  - joint_3
  - joint_4
  - joint_5
  - joint_6
```

## Test
run the node `get_tf_ik.py` providing the name of the IK server and the name of the desired tf. Example:
```bash
# ROS1
rosrun ik_solver get_tf_ik.py <plugin_name> <tf_name>
# ROS2
python3 <path_to_ik_solver_package>/scripts/get_tf_ik_ros2.py <plugin_name> <tf_name>
```
The node publish a `moveit_msgs/DisplayRobotState` topic called `ik_solution`. The robot state shows cyclically the IK solution.
>>>>>>> 42d95c992cd93fcc7bf11e099fe169c0f1b483c2
