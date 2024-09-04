# This Python file uses the following encoding: utf-8

from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node

import yaml
import os
from pathlib import Path

def generate_launch_description():
  launch_arg = [
    DeclareLaunchArgument("config",   description="Path to config file"),
  ]
  return LaunchDescription([*launch_arg, OpaqueFunction(function=launch_setup)])

def launch_setup(context, *args, **kwargs):
  config_path = LaunchConfiguration("config").perform(context)

  if os.path.exists(config_path):
    param_dir = os.environ["CNR_PARAM_ROOT_DIRECTORY"]
    launch_node_after_load = list()
    ret = []

    with open(config_path, 'r') as yaml_file:
      # Open yaml parameter file
      yaml_struct = yaml.safe_load(yaml_file)

      # Redundancy check
      if len(yaml_struct.keys()) != len(set(yaml_struct.keys())):
        print(f'The yaml file {config_path} requires multiple nodes with the same name. Aborting')

      else:
        # Each first level parameter correspond to a new node to launch

        cnr_param_server_process = ExecuteProcess(
          cmd = [[
            FindExecutable(name="cnr_param_server"),
            " --path-to-file ",
            config_path
          ]],
          shell=True,
        )

        for yaml_node in yaml_struct.keys():

          # Ignore ROS parameters
          if 'ros__parameters' in yaml_struct[yaml_node].keys():
            continue

          # Launch ik_solver_node after parameters has been properly loaded
          # Each ik_solver_node has a namespace equal to the 1st-level parameter
          launch_node_after_load.append(RegisterEventHandler(
            OnProcessExit(
              target_action=cnr_param_server_process,
              on_exit=Node(
                            package="ik_solver",
                            executable="ik_solver_node",
                            output="screen",
                            namespace=yaml_node,
                            ros_arguments=["--log-level", "info"],
                            remappings=[(f"/{yaml_node}/ik_solver_node/robot_description", "/robot_description")],
                            # prefix=["gnome-terminal -- gdb -q -ex run --args"],
                          )
            )
          ))

    ret =  [cnr_param_server_process,
            *launch_node_after_load]

  return ret
