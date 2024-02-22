# This Python file uses the following encoding: utf-8

from launch import LaunchDescription, LaunchContext
from launch.actions import ExecuteProcess, OpaqueFunction, DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  launch_arg = [
    DeclareLaunchArgument("plugin"),
    DeclareLaunchArgument("filename")
  ]
  return LaunchDescription([*launch_arg, OpaqueFunction(function=launch_setup)])

def launch_setup(context, *args, **kwargs):
  pack_names = LaunchConfiguration("plugin")
  filenames = LaunchConfiguration("filename")

  pack_splitted = pack_names.perform(context).split(",")
  file_splitted = filenames.perform(context).split(",")
  trans = str.maketrans("","","[' ]")
  trans2 = str.maketrans("","",'"')
  pack_list = [s.translate(trans).translate(trans2) for s in pack_splitted]
  print(pack_list)
  file_list = [s.translate(trans).translate(trans2) for s in file_splitted]
  print(file_list)
  if(len(pack_list) != len(file_list)):
    raise Exception(f"[ERROR]: Arguments: The number of packages is not equal to the number of filenames")

  load_plugin_param_proc = []
  ik_solver_nodes = []
  launch_node_after_load = []

  for pack, filen in zip(pack_list, file_list):
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
      arguments=["--ros-args", "--log-level", "info"]
    ))

    launch_node_after_load.append(RegisterEventHandler(
      OnProcessExit(
        target_action=load_plugin_param_proc[-1],
        on_exit=ik_solver_nodes[-1]
      )
    ))



  return [*load_plugin_param_proc,
          *launch_node_after_load]
