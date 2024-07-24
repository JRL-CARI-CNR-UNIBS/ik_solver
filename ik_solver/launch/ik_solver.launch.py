# This Python file uses the following encoding: utf-8

from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import yaml

__NO_FILE: str = "__empty"

def generate_launch_description():
  launch_arg = [
    DeclareLaunchArgument("file",   description="Path to config file",
                                    default_value=__NO_FILE),
  ]
  return LaunchDescription([*launch_arg, OpaqueFunction(function=launch_setup)])

def launch_setup(context, *args, **kwargs):
  ret = from_config(context)
  return ret

def from_config(context):
  configfile = LaunchConfiguration("file")

  load_plugin_param_proc = list()
  launch_node_after_load = list()

  filename = configfile.perform(context)

  with open(filename, 'r') as yaml_file:
    yaml_struct = yaml.safe_load(yaml_file)
    for el in yaml_struct["ik_solver"]:
      load_plugin_param_proc.append(
        ExecuteProcess(
          cmd = [
            FindExecutable(name="cnr_param_server"),
            "--path-to-file",
            PathJoinSubstitution([
              FindPackageShare(el["package"]),
              el["config"]
            ])
          ],
          shell=False
        )
      )
      ik_solver_node = Node(
        package="ik_solver",
        executable="ik_solver_node",
        output="screen",
        namespace=el["namespace"],
        ros_arguments=["--log-level", "info"],
      )
      launch_node_after_load.append(RegisterEventHandler(
        OnProcessExit(
          target_action=load_plugin_param_proc[-1],
          on_exit=ik_solver_node
        )
      ))

  return [*load_plugin_param_proc,
          *launch_node_after_load]
