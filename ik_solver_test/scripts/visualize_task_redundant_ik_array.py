#!/usr/bin/env python3

import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayRobotState

from ik_solver_msgs.msg import Configuration, IkTarget
from ik_solver_msgs.srv import GetBound, GetFk, GetFrames, GetIkArray


class VisualizeTaskRedundantIkArrayNode(Node):
    def __init__(self, service_name, period):
        super().__init__('visualize_task_redundant_ik_array')
        self.service_name = service_name
        self.period = period

        self.state_pub = self.create_publisher(DisplayRobotState, '/display_robot_state', 10)

        self.bound_client = self.create_client(GetBound, f'/{self.service_name}/get_bounds')
        while not self.bound_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {self.service_name}/get_bounds not available, waiting again...')
        req = GetBound.Request()
        self.future = self.bound_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        res = self.future.result()

        bounds_map = {}
        for bound in res.boundaries:
            bounds_map[bound.joint_name] = (bound.lower_bound, bound.upper_bound)

        joint_names = list(bounds_map.keys())

        self.frames_client = self.create_client(GetFrames, f'/{self.service_name}/get_frames')
        while not self.frames_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {self.service_name}/get_frames not available, waiting again...')
        req = GetFrames.Request()
        self.future = self.frames_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        res = self.future.result()

        base_frame = res.base_frame
        tool_frame = res.tool_frame

        self.fk_client = self.create_client(GetFk, f'/{self.service_name}/get_fk')
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {self.service_name}/get_fk not available, waiting again...')

        fk_req = GetFk.Request()
        fk_req.joint_names = joint_names
        fk_req.reference_frame = base_frame
        fk_req.tip_frame = tool_frame
        for jn in joint_names:
            lb, ub = bounds_map[jn]
            fk_req.configuration.configuration.append(lb + np.random.rand() * (ub - lb))

        self.future = self.fk_client.call_async(fk_req)
        rclpy.spin_until_future_complete(self, self.future)
        fk_res = self.future.result()

        self.task_redundant_client = self.create_client(
            GetIkArray, f'/{self.service_name}/get_task_reduntant_ik_array')
        while not self.task_redundant_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f'Service {self.service_name}/get_task_reduntant_ik_array not available, waiting again...')

        target = IkTarget()
        target.pose = fk_res.pose
        target.seeds = [Configuration(configuration=fk_req.configuration.configuration)]

        req = GetIkArray.Request()
        req.targets = [target]
        req.seed_joint_names = joint_names

        self.future = self.task_redundant_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        res = self.future.result()

        result_joint_names = list(res.joint_names)
        configurations = [
            conf for solution in res.solutions for conf in solution.configurations
        ]
        self.get_logger().info(
            f'received {len(configurations)} configurations across {len(res.solutions)} targets, '
            f'publishing one every {self.period}s on /display_robot_state '
            f'(add a RobotState display in RViz2 subscribed to that topic)')

        for index, conf in enumerate(configurations):
            msg = DisplayRobotState()
            msg.state.joint_state.header.stamp = self.get_clock().now().to_msg()
            msg.state.joint_state.header.frame_id = base_frame
            msg.state.joint_state.name = result_joint_names
            msg.state.joint_state.position = list(conf.configuration)
            msg.state.is_diff = False

            self.state_pub.publish(msg)
            self.get_logger().info(f'published configuration {index + 1}/{len(configurations)}')
            time.sleep(self.period)


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run ik_solver_test visualize_task_redundant_ik_array.py <namespace> [period_s]")
        return

    service_name = sys.argv[1]
    period = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0

    node = VisualizeTaskRedundantIkArrayNode(service_name, period)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
