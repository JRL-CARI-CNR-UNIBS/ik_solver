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
# ROS2
ros2 launch ik_solver ik_solver.launch.py plugin:='[<plugin_pkg_name_1>, <plugin_pkg_name_2>, ...]' config:='[<config_filename_plugin_1>, <config_filename_plugin_2>, ...]'
```

The following arguments are required:

- `plugin`: name of the packages of the plugins which needs to be loaded. This will also provide the namespace the namespace of the node implementing each plugin.
- `config`: config file containing the plugin settings (without `.yaml`).

**Important:** both the previous arguments must be string of arrays and **not** arrays of strings.

The number of plugins required must match the number of filenames. The config file must be in `<plugin_pkg_name>/config/<config_filename_plugin>`.

#### Example
```bash
ros2 launch ik_solver ik_solver.launch.py plugin:='[rosdyn_ik_solver, comau_ik_solver]' config:='[ik_solver, generic]'
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
