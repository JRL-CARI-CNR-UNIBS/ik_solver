#!/usr/bin/env python3
"""
IK Visualizer Script for RViz.

Supports both:
- Task-Redundant IK solver (get_task_reduntant_ik_array / ik_task_redundant)
- Base IK solver (get_ik)

This script samples a random joint configuration, calls the selected IK service,
and visualizes the resulting configurations in RViz.

Pressing SPACE on the keyboard steps through the solutions and, once exhausted,
automatically requests a new random configuration. You can also press 'n' to
sample a new random target configuration immediately, or 'm' / 't' to toggle
between Base IK and Task-Redundant IK solvers at runtime.
"""

import argparse
import atexit
import os
import random
import select
import sys
import termios
import threading
import time
import tty
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

import geometry_msgs.msg
import moveit_msgs.msg
import sensor_msgs.msg
from ik_solver_msgs.msg import Configuration, IkTarget
from ik_solver_msgs.srv import GetBound, GetFk, GetFrames, GetIk, GetIkArray


class RawTerminal:
    """Context manager and helper for reading single key presses non-blockingly."""

    def __init__(self):
        self.fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        self.old_settings = None
        if self.fd is not None:
            self.old_settings = termios.tcgetattr(self.fd)
            atexit.register(self.restore)

    def set_cbreak(self):
        if self.fd is not None:
            tty.setcbreak(self.fd)

    def restore(self):
        if self.fd is not None and self.old_settings is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
            except Exception:
                pass

    def get_key(self, timeout_sec: float = 0.05) -> Optional[str]:
        if self.fd is None:
            return None
        rlist, _, _ = select.select([sys.stdin], [], [], timeout_sec)
        if rlist:
            char = sys.stdin.read(1)
            # Handle escape sequences (e.g. arrow keys)
            if char == '\x1b':
                rlist2, _, _ = select.select([sys.stdin], [], [], 0.01)
                if rlist2:
                    char += sys.stdin.read(2)
            return char
        return None


class TaskRedundantIkVisualizerNode(Node):
    """ROS 2 Node to query IK (base or task-redundant) and publish configurations to RViz."""

    def __init__(
        self,
        namespace: str = "",
        mode: str = "task_redundant",
        service_name: Optional[str] = None,
        bounds_service: Optional[str] = None,
        frames_service: Optional[str] = None,
        fk_service: Optional[str] = None,
        heartbeat_rate_hz: float = 10.0,
    ):
        super().__init__('ik_visualizer')
        self.namespace = namespace.strip('/')
        self.mode = mode.lower()  # 'task_redundant' or 'base'
        self.service_override = service_name
        self.bounds_service_override = bounds_service
        self.frames_service_override = frames_service
        self.fk_service_override = fk_service

        # Publishers for RViz
        self.display_state_pub = self.create_publisher(
            moveit_msgs.msg.DisplayRobotState, '/display_robot_state', 10
        )
        self.ik_solution_pub = self.create_publisher(
            moveit_msgs.msg.DisplayRobotState, '/ik_solution', 10
        )
        self.planning_scene_pub = self.create_publisher(
            moveit_msgs.msg.PlanningScene, '/planning_scene', 10
        )
        self.joint_state_pub = self.create_publisher(
            sensor_msgs.msg.JointState, '/joint_states', 10
        )

        # Discovered parameters and service clients
        self.joint_names: List[str] = []
        self.bounds_map: dict[str, Tuple[float, float]] = {}
        self.base_frame: str = 'base_link'
        self.tool_frame: str = 'flange'

        self.bound_client = None
        self.frames_client = None
        self.fk_client = None
        self.task_redundant_client = None
        self.base_ik_client = None

        # State management for solutions
        self.current_configurations: List[List[float]] = []
        self.current_index: int = -1
        self.current_target_pose: Optional[geometry_msgs.msg.PoseStamped] = None
        self.current_seed: Optional[List[float]] = None
        self.active_config: Optional[List[float]] = None
        self.lock = threading.Lock()

        # Timer for continuous republishing (keeps RViz rendering stable)
        if heartbeat_rate_hz > 0.0:
            self.timer = self.create_timer(1.0 / heartbeat_rate_hz, self._heartbeat_callback)
        else:
            self.timer = None

    def _resolve_service_name(self, candidate_names: List[str], target_type: str) -> Optional[str]:
        """Find an existing service name matching one of candidate_names."""
        active_services = dict(self.get_service_names_and_types())
        for cand in candidate_names:
            full_cand = f'/{cand}'.replace('//', '/')
            if full_cand in active_services:
                return full_cand

        for cand in candidate_names:
            base_cand = cand.split('/')[-1]
            for srv_name, srv_types in active_services.items():
                if srv_name.endswith(f'/{base_cand}') and target_type in srv_types:
                    return srv_name

        return f'/{candidate_names[0]}'.replace('//', '/')

    def discover_services(self) -> bool:
        """Discover and initialize service clients."""
        ns_prefix = f'{self.namespace}/' if self.namespace else ''

        # 1. Bounds service
        bounds_srv_name = (
            self.bounds_service_override
            if self.bounds_service_override
            else self._resolve_service_name(
                [f'{ns_prefix}get_bounds', 'get_bounds'], 'ik_solver_msgs/srv/GetBound'
            )
        )

        # 2. Frames service
        frames_srv_name = (
            self.frames_service_override
            if self.frames_service_override
            else self._resolve_service_name(
                [f'{ns_prefix}get_frames', 'get_frames'], 'ik_solver_msgs/srv/GetFrames'
            )
        )

        # 3. FK service
        fk_srv_name = (
            self.fk_service_override
            if self.fk_service_override
            else self._resolve_service_name(
                [f'{ns_prefix}get_fk', 'get_fk'], 'ik_solver_msgs/srv/GetFk'
            )
        )

        self.get_logger().info(f"Using bounds service: '{bounds_srv_name}'")
        self.get_logger().info(f"Using frames service: '{frames_srv_name}'")
        self.get_logger().info(f"Using FK service:     '{fk_srv_name}'")

        self.bound_client = self.create_client(GetBound, bounds_srv_name)
        self.frames_client = self.create_client(GetFrames, frames_srv_name)
        self.fk_client = self.create_client(GetFk, fk_srv_name)

        # Wait for common services
        self.get_logger().info("Waiting for common ik_solver services to be available...")
        for name, client in [
            (bounds_srv_name, self.bound_client),
            (frames_srv_name, self.frames_client),
            (fk_srv_name, self.fk_client),
        ]:
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"Timed out waiting for service '{name}'.")
                return False

        # 4. Discover IK services (try discovering both for dynamic switching)
        # Task-redundant IK service
        redundant_srv_name = None
        if self.mode == "task_redundant" and self.service_override:
            redundant_srv_name = self.service_override
            if not redundant_srv_name.startswith('/'):
                redundant_srv_name = f'/{ns_prefix}{redundant_srv_name}'
        else:
            redundant_candidates = [
                f'{ns_prefix}get_task_reduntant_ik_array',
                f'{ns_prefix}get_task_redundant_ik_array',
                f'{ns_prefix}ik_task_redundant',
                'get_task_reduntant_ik_array',
                'get_task_redundant_ik_array',
                'ik_task_redundant',
            ]
            redundant_srv_name = self._resolve_service_name(
                redundant_candidates, 'ik_solver_msgs/srv/GetIkArray'
            )

        # Base IK service
        base_srv_name = None
        if self.mode == "base" and self.service_override:
            base_srv_name = self.service_override
            if not base_srv_name.startswith('/'):
                base_srv_name = f'/{ns_prefix}{base_srv_name}'
        else:
            base_candidates = [
                f'{ns_prefix}get_ik',
                'get_ik',
            ]
            base_srv_name = self._resolve_service_name(
                base_candidates, 'ik_solver_msgs/srv/GetIk'
            )

        self.get_logger().info(f"Using redundant IK service: '{redundant_srv_name}'")
        self.get_logger().info(f"Using base IK service:      '{base_srv_name}'")

        self.task_redundant_client = self.create_client(GetIkArray, redundant_srv_name)
        self.base_ik_client = self.create_client(GetIk, base_srv_name)

        # Check required client based on active mode
        if self.mode == "base":
            if not self.base_ik_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"Timed out waiting for base IK service '{base_srv_name}'.")
                return False
        else:
            if not self.task_redundant_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(
                    f"Timed out waiting for task-redundant IK service '{redundant_srv_name}'."
                )
                return False

        self.get_logger().info(f"Initialized services in mode: '{self.mode}'")
        return True

    def initialize_robot_info(self) -> bool:
        """Fetch joint boundaries and coordinate frames from ik_solver."""
        # 1. Fetch boundaries
        req_bounds = GetBound.Request()
        future = self.bound_client.call_async(req_bounds)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            self.get_logger().error("Failed to retrieve joint boundaries from get_bounds.")
            return False

        res_bounds = future.result()
        self.bounds_map.clear()
        for bound in res_bounds.boundaries:
            self.bounds_map[bound.joint_name] = (bound.lower_bound, bound.upper_bound)
        self.joint_names = list(self.bounds_map.keys())

        if not self.joint_names:
            self.get_logger().error("Joint bounds list is empty.")
            return False

        # 2. Fetch frames
        req_frames = GetFrames.Request()
        future = self.frames_client.call_async(req_frames)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            self.get_logger().warn("Failed to retrieve frames from get_frames, using defaults.")
            self.base_frame = 'base_link'
            self.tool_frame = 'flange'
        else:
            res_frames = future.result()
            self.base_frame = res_frames.base_frame or 'base_link'
            self.tool_frame = res_frames.tool_frame or 'flange'

        self.get_logger().info(f"Loaded joints ({len(self.joint_names)}): {self.joint_names}")
        self.get_logger().info(f"Base frame: '{self.base_frame}', Tool frame: '{self.tool_frame}'")
        return True

    def sample_random_configuration(self) -> List[float]:
        """Generate a random joint configuration strictly within bounds."""
        config = []
        for jn in self.joint_names:
            lb, ub = self.bounds_map[jn]
            if lb == ub:
                val = lb
            else:
                val = lb + random.random() * (ub - lb)
            config.append(val)
        return config

    def toggle_mode(self):
        """Toggle between Base IK and Task-Redundant IK solvers."""
        if self.mode == "base":
            if self.task_redundant_client.service_is_ready():
                self.mode = "task_redundant"
            else:
                print("\n[WARNING] Task-redundant IK service is not available, keeping Base IK mode.")
                return
        else:
            if self.base_ik_client.service_is_ready():
                self.mode = "base"
            else:
                print("\n[WARNING] Base IK service is not available, keeping Task-Redundant IK mode.")
                return

        mode_name = "Task-Redundant IK" if self.mode == "task_redundant" else "Base IK"
        print(f"\n[MODE SWITCH] Switched solver to: >>> {mode_name} <<<")
        self.request_ik()

    def request_ik(self) -> bool:
        """Sample a random configuration, compute FK, and call active IK service."""
        # 1. Sample random configuration
        random_config = self.sample_random_configuration()
        self.current_seed = random_config

        # 2. Compute FK for target pose
        fk_req = GetFk.Request()
        fk_req.joint_names = self.joint_names
        fk_req.reference_frame = self.base_frame
        fk_req.tip_frame = self.tool_frame
        fk_req.configuration.configuration = list(random_config)

        future_fk = self.fk_client.call_async(fk_req)
        rclpy.spin_until_future_complete(self, future_fk, timeout_sec=5.0)
        if not future_fk.done() or future_fk.result() is None:
            self.get_logger().error("FK service call failed.")
            return False
        fk_res = future_fk.result()
        self.current_target_pose = fk_res.pose

        target = IkTarget()
        target.pose = fk_res.pose
        target.seeds = [Configuration(configuration=list(random_config))]

        # 3. Call active IK service
        solutions: List[List[float]] = []

        if self.mode == "base":
            ik_req = GetIk.Request()
            ik_req.target = target
            ik_req.seed_joint_names = self.joint_names

            future_ik = self.base_ik_client.call_async(ik_req)
            rclpy.spin_until_future_complete(self, future_ik, timeout_sec=5.0)
            if not future_ik.done() or future_ik.result() is None:
                self.get_logger().error("Base IK service call failed.")
                return False

            ik_res = future_ik.result()
            if ik_res.joint_names:
                self.joint_names = list(ik_res.joint_names)

            solutions = [
                list(conf.configuration)
                for conf in ik_res.solution.configurations
            ]
        else:
            # Task-redundant IK
            ik_req = GetIkArray.Request()
            ik_req.targets = [target]
            ik_req.seed_joint_names = self.joint_names

            future_ik = self.task_redundant_client.call_async(ik_req)
            rclpy.spin_until_future_complete(self, future_ik, timeout_sec=10.0)
            if not future_ik.done() or future_ik.result() is None:
                self.get_logger().error("Task-redundant IK service call failed.")
                return False

            ik_res = future_ik.result()
            if ik_res.joint_names:
                self.joint_names = list(ik_res.joint_names)

            solutions = [
                list(conf.configuration)
                for sol in ik_res.solutions
                for conf in sol.configurations
            ]

        with self.lock:
            self.current_configurations = solutions
            self.current_index = 0 if solutions else -1
            if solutions:
                self.active_config = solutions[0]
            else:
                self.active_config = None

        p = fk_res.pose.pose.position
        pos_str = f"({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
        mode_label = "Base IK" if self.mode == "base" else "Task-Redundant IK"
        if solutions:
            print(f"\n[{mode_label}] Target Pos: {pos_str} -> Found {len(solutions)} solution(s).")
            self.publish_current_state()
        else:
            print(f"\n[{mode_label}] Target Pos: {pos_str} -> 0 solutions found. Press SPACE to sample another.")

        return True

    def show_next_configuration(self):
        """Step to the next solution configuration; if at end, request new random target."""
        with self.lock:
            num_confs = len(self.current_configurations)
            if num_confs == 0 or self.current_index + 1 >= num_confs:
                need_new = True
            else:
                self.current_index += 1
                self.active_config = self.current_configurations[self.current_index]
                need_new = False

        if need_new:
            print("\n[INFO] End of solution set reached. Generating new random configuration...")
            self.request_ik()
        else:
            self.publish_current_state()

    def show_prev_configuration(self):
        """Step to the previous solution configuration."""
        with self.lock:
            num_confs = len(self.current_configurations)
            if num_confs == 0:
                print("\n[INFO] No solutions loaded. Press SPACE to generate a new target.")
                return
            if self.current_index > 0:
                self.current_index -= 1
                self.active_config = self.current_configurations[self.current_index]
            else:
                print(f"\n[INFO] Already at the first configuration (1/{num_confs}).")

        self.publish_current_state()

    def publish_current_state(self):
        """Publish the active configuration to RViz topics."""
        with self.lock:
            if self.active_config is None:
                return
            config = list(self.active_config)
            idx = self.current_index
            total = len(self.current_configurations)

        self._publish_messages(config)

        angles_str = ", ".join([f"{v:.3f}" for v in config])
        print(f" -> Showing configuration [{idx + 1}/{total}]: [{angles_str}]")

    def _publish_messages(self, config: List[float]):
        """Publish ROS messages to /display_robot_state, /planning_scene, and /joint_states."""
        now = self.get_clock().now().to_msg()

        # 1. DisplayRobotState message
        drs_msg = moveit_msgs.msg.DisplayRobotState()
        drs_msg.state.is_diff = False
        drs_msg.state.joint_state.header.stamp = now
        drs_msg.state.joint_state.header.frame_id = self.base_frame
        drs_msg.state.joint_state.name = self.joint_names
        drs_msg.state.joint_state.position = config
        self.display_state_pub.publish(drs_msg)
        self.ik_solution_pub.publish(drs_msg)

        # 2. PlanningScene diff message (updates MoveIt RViz PlanningScene / MotionPlanning display)
        scene_msg = moveit_msgs.msg.PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state.is_diff = True
        scene_msg.robot_state.joint_state.header.stamp = now
        scene_msg.robot_state.joint_state.header.frame_id = self.base_frame
        scene_msg.robot_state.joint_state.name = self.joint_names
        scene_msg.robot_state.joint_state.position = config
        self.planning_scene_pub.publish(scene_msg)

        # 3. JointState message (updates standard RobotModel display)
        js_msg = sensor_msgs.msg.JointState()
        js_msg.header.stamp = now
        js_msg.header.frame_id = self.base_frame
        js_msg.name = self.joint_names
        js_msg.position = config
        self.joint_state_pub.publish(js_msg)

    def _heartbeat_callback(self):
        """Periodically republish active configuration to keep RViz display alive."""
        with self.lock:
            if self.active_config is None:
                return
            config = list(self.active_config)
        self._publish_messages(config)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="RViz Visualizer for IK solutions (Base IK or Task-Redundant IK)."
    )
    parser.add_argument(
        'namespace_pos',
        nargs='?',
        default=None,
        help="Optional positional solver namespace (e.g. 'solver1' or 'kuka_ik_solver')",
    )
    parser.add_argument(
        '-m', '--mode',
        type=str,
        choices=['redundant', 'task_redundant', 'base', 'standard'],
        default='redundant',
        help="IK solver type: 'redundant' (Task-Redundant IK, default) or 'base' (Standard Base IK)",
    )
    parser.add_argument(
        '--base',
        action='store_true',
        help="Shortcut to use Base IK solver (equivalent to --mode base)",
    )
    parser.add_argument(
        '--redundant',
        action='store_true',
        help="Shortcut to use Task-Redundant IK solver (equivalent to --mode redundant)",
    )
    parser.add_argument(
        '-n', '--namespace',
        type=str,
        default=None,
        help="Solver namespace (e.g. 'solver1')",
    )
    parser.add_argument(
        '-s', '--service',
        type=str,
        default=None,
        help="Explicit IK service name (e.g. '/solver1/get_task_reduntant_ik_array' or '/solver1/get_ik')",
    )
    parser.add_argument(
        '--bounds-service',
        type=str,
        default=None,
        help="Explicit get_bounds service name",
    )
    parser.add_argument(
        '--frames-service',
        type=str,
        default=None,
        help="Explicit get_frames service name",
    )
    parser.add_argument(
        '--fk-service',
        type=str,
        default=None,
        help="Explicit get_fk service name",
    )
    parser.add_argument(
        '--rate',
        type=float,
        default=10.0,
        help="RViz state republish rate in Hz (default: 10.0)",
    )
    return parser.parse_args()


def print_banner(mode: str):
    mode_name = "Task-Redundant IK" if mode == "task_redundant" else "Base IK"
    print("=" * 70)
    print(f"        IK Visualizer for RViz [{mode_name}]")
    print("=" * 70)
    print(" Controls:")
    print("   [SPACE]     : Show NEXT configuration (samples new target at end)")
    print("   [n / r]     : Sample a NEW random joint configuration and solve")
    print("   [m / t]     : TOGGLE solver mode (Base IK <-> Task-Redundant IK)")
    print("   [p / b]     : Show PREVIOUS configuration")
    print("   [q/ESC/C-c] : Quit")
    print("=" * 70)


def main(args=None):
    rclpy.init(args=args)
    parsed = parse_arguments()

    # Determine namespace
    namespace = parsed.namespace or parsed.namespace_pos or ""

    # Determine solver mode
    if parsed.base:
        mode = "base"
    elif parsed.redundant:
        mode = "task_redundant"
    elif parsed.mode in ("base", "standard"):
        mode = "base"
    else:
        mode = "task_redundant"

    node = TaskRedundantIkVisualizerNode(
        namespace=namespace,
        mode=mode,
        service_name=parsed.service,
        bounds_service=parsed.bounds_service,
        frames_service=parsed.frames_service,
        fk_service=parsed.fk_service,
        heartbeat_rate_hz=parsed.rate,
    )

    if not node.discover_services():
        node.get_logger().error("Service discovery failed. Exiting.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if not node.initialize_robot_info():
        node.get_logger().error("Failed to initialize robot parameters. Exiting.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print_banner(node.mode)

    # Spin the node in a background thread for ROS 2 callbacks / timers
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Initial request
    print(f"Requesting initial solutions using {node.mode} mode...")
    node.request_ik()

    terminal = RawTerminal()
    terminal.set_cbreak()

    try:
        while rclpy.ok():
            key = terminal.get_key(timeout_sec=0.05)
            if key is None:
                continue

            # SPACE key: Next configuration
            if key == ' ':
                node.show_next_configuration()

            # 'n' or 'r': New random target
            elif key in ('n', 'N', 'r', 'R'):
                print("\n[USER] Sampling new random target configuration...")
                node.request_ik()

            # 'm' or 't': Toggle mode between Base IK and Task-Redundant IK
            elif key in ('m', 'M', 't', 'T'):
                node.toggle_mode()

            # 'p' or 'b' or Up/Left arrow: Previous configuration
            elif key in ('p', 'P', 'b', 'B', '\x1b[A', '\x1b[D'):
                node.show_prev_configuration()

            # Down/Right arrow: Next configuration
            elif key in ('\x1b[B', '\x1b[C'):
                node.show_next_configuration()

            # Quit keys
            elif key in ('q', 'Q', '\x1b', '\x03'):
                print("\nExiting visualizer...")
                break

    except KeyboardInterrupt:
        print("\nInterrupted by user. Exiting...")
    finally:
        terminal.restore()
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
