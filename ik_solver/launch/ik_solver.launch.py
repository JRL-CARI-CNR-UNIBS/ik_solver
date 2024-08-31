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
    prefix_path = Path(param_dir) / 'ik_solver_param'
    load_plugin_param_proc = list()
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
        for yaml_node in yaml_struct.keys():

          # Check directory
          if not os.path.isdir(prefix_path):
            os.makedirs(prefix_path)

          # Temporary parameter file to save the portion of yaml that correspond to a single node
          segment_file = prefix_path / yaml_node
          with open(segment_file, 'w') as tmp:
            yaml.dump({yaml_node : yaml_struct[yaml_node]}, tmp)

          # Load parameters of the node into cnr parameter server
          load_plugin_param_proc.append(
            ExecuteProcess(
              cmd = [[
                FindExecutable(name="cnr_param_server"),
                " --path-to-file ",
                segment_file.as_posix()
              ]],
              shell=True,
            )
          )

          # Launch ik_solver_node after parameters has been properly loaded
          # Each ik_solver_node has a namespace equal to the 1st-level parameter
          launch_node_after_load.append(RegisterEventHandler(
            OnProcessExit(
              target_action=load_plugin_param_proc[-1],
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

    ret =  [*load_plugin_param_proc,
            *launch_node_after_load]

  return ret
