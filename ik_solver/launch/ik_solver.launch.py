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
    DeclareLaunchArgument("plugin", description="String of array of plugin names",
                                    default_value="null"),
    DeclareLaunchArgument("config", description="String of array of config files (with or without '.yaml')",
                                    default_value="null"),
    DeclareLaunchArgument("file",   description="Path to config file",
                                    default_value=__NO_FILE),
  ]
  return LaunchDescription([*launch_arg, OpaqueFunction(function=launch_setup)])

def launch_setup(context, *args, **kwargs):
  configfile = LaunchConfiguration("file")

  ret: list
  if(configfile.perform(context) != __NO_FILE):
    ret = from_config(context)
  else:
    ret = from_list(context)

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

def from_list(context):
  import os
  print("=== WARNING: Prefer the use of 'file' argument ===")
  pack_names = LaunchConfiguration("plugin")
  filenames = LaunchConfiguration("config")
  pack_splitted = pack_names.perform(context).split(",")
  file_splitted = filenames.perform(context).split(",")
  trans = str.maketrans("","","[' ]")
  trans2 = str.maketrans("","",'"')
  pack_list = [s.translate(trans).translate(trans2) for s in pack_splitted]
  print(pack_list)
  file_list = [s.translate(trans).translate(trans2) for s in file_splitted]
  print(file_list)
  if(len(pack_list) != len(file_list)):
    raise Exception("[ERROR]: Arguments: The number of packages is not equal to the number of filenames")

  load_plugin_param_proc = []
  ik_solver_nodes = []
  launch_node_after_load = []

  for pack, filen in zip(pack_list, file_list):
    if(os.path.splitext(filen)[1] != ".yaml"):
      filen += ".yaml"

    load_plugin_param_proc.append(
      ExecuteProcess(
        cmd = [
          FindExecutable(name="cnr_param_server"),
          "--path-to-file",
          PathJoinSubstitution([
            FindPackageShare(pack),
            "config",
            filen
          ])
        ],
        shell=False
      )
    )

    ik_solver_nodes.append(Node(
      package="ik_solver",
      executable="ik_solver_node",
      output="screen",
      namespace=pack,
      ros_arguments=["--log-level", "info"],
    ))

    launch_node_after_load.append(RegisterEventHandler(
      OnProcessExit(
        target_action=load_plugin_param_proc[-1],
        on_exit=ik_solver_nodes[-1]
      )
    ))

  return [*load_plugin_param_proc,
          *launch_node_after_load]
