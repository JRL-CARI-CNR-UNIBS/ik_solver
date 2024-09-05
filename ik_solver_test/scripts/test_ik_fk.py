#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from moveit_msgs.msg import DisplayRobotState
from ik_solver_msgs.srv import GetIk
from ik_solver_msgs.srv import GetFk
from ik_solver_msgs.srv import GetBound
from ik_solver_msgs.srv import GetFrames
import sys
import numpy as np

from requests.utils import from_key_val_list
from tables.idxutils import infinity


class GetIkNode(Node):
    def __init__(self, service_name, tf_name):
        super().__init__('get_ik')
        self.service_name = service_name
        self.tf_name = tf_name

        self.state_pub = self.create_publisher(DisplayRobotState, '/ik_solution', 10)

        self.bound_client = self.create_client(GetBound, f'/{self.service_name}/get_bounds')
        while not self.bound_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {self.service_name}/get_bounds not available, waiting again...')
        req = GetBound.Request()
        self.future = self.bound_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        res= self.future.result()

        bounds_map={}

        for bound in res.boundaries:
            bounds_map[bound.joint_name]=(bound.lower_bound,bound.upper_bound)

        self.get_logger().info(f'Bounds  {bounds_map}')
        joint_names=list(bounds_map.keys())
        self.get_logger().info(f'Joints  {joint_names}')

        self.frames_client = self.create_client(GetFrames, f'/{self.service_name}/get_frames')
        while not self.frames_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {self.service_name}/get_frames not available, waiting again...')
        req = GetFrames.Request()
        self.future = self.frames_client.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        res= self.future.result()

        self.base_frame = res.base_frame
        self.tool_frame = res.tool_frame


        self.fk_client = self.create_client(GetFk, f'/{self.service_name}/get_fk')
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {self.service_name}/get_fk not available, waiting again...')

        n_errors=0
        n_trials=1000
        for index in range(0,n_trials):
            self.get_logger().info(f'run {index} of {n_trials} with {n_errors} errors...')

            #if index%10 == 0:
            #    self.get_logger().info(f'run {index} of {n_trials} with {n_errors} errors...')
            self.fk_req = GetFk.Request()
            self.fk_req.joint_names = joint_names
            self.fk_req.reference_frame = self.base_frame
            self.fk_req.tip_frame = self.tool_frame

            for js in joint_names:
                lb=bounds_map[js][0]
                ub = bounds_map[js][1]
                self.fk_req.configuration.configuration.append( lb+np.random.rand()*(ub-lb)  )
            self.future = self.fk_client.call_async(self.fk_req)
            rclpy.spin_until_future_complete(self, self.future)
            fk_res = self.future.result()

            self.ik_client = self.create_client(GetIk, f'/{self.service_name}/get_ik')
            while not self.ik_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Service {self.service_name}/get_ik not available, waiting again...')

            self.req = GetIk.Request()
            self.req.target.pose=fk_res.pose

            self.future = self.ik_client.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
            ik_res = self.future.result()

            q = np.array(self.fk_req.configuration.configuration)

            norma = infinity
            for conf in ik_res.solution.configurations:
                q2 = np.array(conf.configuration)
                difference = q - q2
                if np.linalg.norm(difference) < norma:
                    norma = np.linalg.norm(difference)

            if norma > 1e-6:
                if max(list(ik_res.solution.translation_residual_errors))>1e-6 or max(list(ik_res.solution.rotation_residual_errors))>=1e-6:
                    n_errors += 1
                    self.get_logger().error(f"error = {norma}")
                    self.get_logger().error(f"translation_residual_errors {max(list(ik_res.solution.translation_residual_errors))} rotation_residual_errors {max(list(ik_res.solution.rotation_residual_errors))} ")

        self.get_logger().info(f'run {n_trials} times with {n_errors} errors...')

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print("Usage: ros2 run ik_solver get_tf_ik [namespace] [tf_name]")
        return

    service_name = sys.argv[1]
    tf_name = sys.argv[2]

    node = GetIkNode(service_name, tf_name)
    #rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
