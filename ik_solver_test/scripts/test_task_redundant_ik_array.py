#!/usr/bin/env python3

import sys

import numpy as np
import rclpy
from rclpy.node import Node

from ik_solver_msgs.msg import Configuration, IkTarget
from ik_solver_msgs.srv import GetBound, GetFk, GetFrames, GetIkArray


class GetTaskRedundantIkArrayNode(Node):
    def __init__(self, service_name):
        super().__init__('get_task_redundant_ik_array')
        self.service_name = service_name

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
        self.get_logger().info(f'Joints  {joint_names}')

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

        self.get_logger().info(f'joint_names: {list(res.joint_names)}')
        self.get_logger().info(f'received {len(res.solutions)} solution sets (one per pose perturbation)')

        n_errors = 0
        for index, solution in enumerate(res.solutions):
            n_solutions = len(solution.configurations)
            if n_solutions == 0:
                n_errors += 1
                self.get_logger().error(f'perturbation {index}: no solution found')
                continue

            max_tra_err = max(solution.translation_residual_errors) if solution.translation_residual_errors else None
            max_rot_err = max(solution.rotation_residual_errors) if solution.rotation_residual_errors else None
            self.get_logger().info(
                f'perturbation {index}: {n_solutions} solutions, '
                f'max translation error: {max_tra_err}, max rotation error: {max_rot_err}')

        self.get_logger().info(f'done: {len(res.solutions)} perturbations, {n_errors} with no solution')


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run ik_solver_test test_task_redundant_ik_array.py <namespace>")
        return

    service_name = sys.argv[1]

    node = GetTaskRedundantIkArrayNode(service_name)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
