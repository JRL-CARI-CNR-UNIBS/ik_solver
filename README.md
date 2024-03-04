# IK Solver

IkSolver is a solver-agnostic interface for inverse kinematics solvers, both numerical and analytical. It is compatible with both ROS1 and ROS2, and a ROS-free interface can be extended to non-ROS use.

The plugin feature of ROS are used to provide the required solver to the program.

The library implements parallel computation routines.

## Dependencies

A `.repo` file compatible with `vcstool` is provided with all the dependencies

- `cnr_param`: ROS-free parameter handling
- `cnr_logger`: ROS-free logger library

## ROS

The following services are provided through the "ik_solver_node":

- `get_ik`: calls the computation of ik related to a single pose
- `get_ik_array`: calls the computation of ik related to a multiple poses
- `get_fk`: calls the computation of the forward kinematics for a single configuration
- `get_fk_array`: calls the computation of the forward kinematics for multiple configuration

Several utility services are provided:

- `get_bounds`
- `change_tool`: change the tool frame

### Use

```bash
# ROS1
###### WIP...
# ROS2
ros2 launch ik_solver ik_solver.launch.py plugin:='[<plugin_pkg_name_1>, <plugin_pkg_name_2>, ...]' filename:='[<config_filename_plugin_1>, <config_filename_plugin_2>, ...]'
```

The following arguments are required:

- `plugin`: name of the packages of the plugins which needs to be loaded. This will also provide the namespace the namespace of the node implementing each plugin.
- `filename`: config file containing the plugin settings.

The number of plugins required must match the number of filenames. The config file must be in `<plugin_pkg_name>/config/<config_filename_plugin>`.


### Environment Variables
```bash
export CNR_PARAM_ROOT_DIRECTORY="/tmp"
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
