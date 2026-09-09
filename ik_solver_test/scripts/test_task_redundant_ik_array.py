#!/usr/bin/env python3

import sys

import numpy as np
import rclpy
from rclpy.node import Node

from ik_solver_msgs.msg import Configuration, IkTarget
from ik_solver_msgs.srv import GetBound, GetFk, GetFrames, GetIkArray


class GetTaskRedundantIkArrayNode(Node):
    def __init__(self, service_name, num_targets=2):
        super().__init__('get_task_redundant_ik_array_test')
        self.service_name = service_name
        self.num_targets = num_targets

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
        self.get_logger().info(f'Joints: {joint_names}')

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

        # Generate multiple targets
        targets = []
        for t_idx in range(self.num_targets):
            fk_req = GetFk.Request()
            fk_req.joint_names = joint_names
            fk_req.reference_frame = base_frame
            fk_req.tip_frame = tool_frame
            sample_config = []
            for jn in joint_names:
                lb, ub = bounds_map[jn]
                sample_config.append(lb + np.random.rand() * (ub - lb))
            fk_req.configuration.configuration = sample_config

            self.future = self.fk_client.call_async(fk_req)
            rclpy.spin_until_future_complete(self, self.future)
            fk_res = self.future.result()

            target = IkTarget()
            target.pose = fk_res.pose
            target.seeds = [Configuration(configuration=sample_config)]
            targets.append(target)

        self.task_redundant_client = self.create_client(
            GetIkArray, f'/{self.service_name}/get_task_reduntant_ik_array')
        while not self.task_redundant_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f'Service {self.service_name}/get_task_reduntant_ik_array not available, waiting again...')

        ik_req = GetIkArray.Request()
        ik_req.targets = targets
        ik_req.seed_joint_names = joint_names

        self.get_logger().info(f'Sending task-redundant IK request with {len(targets)} targets...')
        self.future = self.task_redundant_client.call_async(ik_req)
        rclpy.spin_until_future_complete(self, self.future)
        ik_res = self.future.result()

        # Format checks
        assert len(ik_res.solutions) == len(targets), (
            f"Output format mismatch: expected {len(targets)} solutions (one per target), "
            f"but got {len(ik_res.solutions)}"
        )
        self.get_logger().info(
            f'SUCCESS: output array length matches input targets count: {len(ik_res.solutions)} == {len(targets)}'
        )

        for target_idx, solution in enumerate(ik_res.solutions):
            n_confs = len(solution.configurations)
            n_tra_errs = len(solution.translation_residual_errors)
            n_rot_errs = len(solution.rotation_residual_errors)

            self.get_logger().info(
                f'Target {target_idx}: {n_confs} total solutions accumulated across perturbations'
            )

            # Verify residual error arrays are consistent with configurations count
            if n_tra_errs > 0 or n_rot_errs > 0:
                assert n_tra_errs == n_confs, (
                    f"Target {target_idx}: translation errors count ({n_tra_errs}) != configurations count ({n_confs})"
                )
                assert n_rot_errs == n_confs, (
                    f"Target {target_idx}: rotation errors count ({n_rot_errs}) != configurations count ({n_confs})"
                )

            if n_confs > 0:
                max_tra_err = max(solution.translation_residual_errors) if solution.translation_residual_errors else None
                max_rot_err = max(solution.rotation_residual_errors) if solution.rotation_residual_errors else None
                self.get_logger().info(
                    f'  -> Max translation error: {max_tra_err}, max rotation error: {max_rot_err}'
                )
            else:
                self.get_logger().warn(f'  -> Warning: No solutions found for Target {target_idx}')


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run ik_solver_test test_task_redundant_ik_array.py <namespace> [num_targets]")
        return

    service_name = sys.argv[1]
    num_targets = int(sys.argv[2]) if len(sys.argv) > 2 else 2

    node = GetTaskRedundantIkArrayNode(service_name, num_targets=num_targets)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
