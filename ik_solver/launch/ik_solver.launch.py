# This Python file uses the following encoding: utf-8
"""
Start one ik_solver_node for each node in a ROS 2 parameter file.

    ros2 launch ik_solver ik_solver.launch.py config:=/path/to/config.yaml

Each first-level key of the file is the fully qualified name of a node,
/<namespace>/ik_solver_node, and the node is started in <namespace> with the
parameters of the file:

    /ur_ik_solver/ik_solver_node:
      ros__parameters:
        type: ik_solver/Ur10eIkSolver
        base_frame: base_link
        ...

Keys that are not of this form (e.g. /**) are not nodes: their parameters are
still given to every node started by this file.
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_arg = [
        DeclareLaunchArgument("config", description="ROS 2 parameter file of the ik_solver nodes"),
    ]
    return LaunchDescription([*launch_arg, OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("config").perform(context)

    if not os.path.exists(config_path):
        raise RuntimeError(f"ik_solver.launch.py: config file {config_path} does not exist")

    with open(config_path, "r") as yaml_file:
        yaml_struct = yaml.safe_load(yaml_file) or {}

    nodes = []
    for key in yaml_struct.keys():
        parts = [p for p in str(key).split("/") if p]
        if len(parts) < 2 or "*" in str(key):
            continue
        namespace = "/".join(parts[:-1])
        name = parts[-1]
        nodes.append(
            Node(
                package="ik_solver",
                executable="ik_solver_node",
                name=name,
                namespace=namespace,
                output="screen",
                parameters=[config_path],
                ros_arguments=["--log-level", "info"],
                remappings=[(f"/{namespace}/{name}/robot_description", "/robot_description")],
            )
        )

    if not nodes:
        raise RuntimeError(
            f"ik_solver.launch.py: no node in {config_path}: the first-level keys must be "
            "/<namespace>/ik_solver_node (ROS 2 parameter file)"
        )
    return nodes
