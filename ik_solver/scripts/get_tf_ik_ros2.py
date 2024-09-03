#!/usr/bin/env python3
import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
import moveit_msgs.msg
import ik_solver_msgs.srv
import sys


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node('get_ik')

    if len(sys.argv)<3:
        node.get_logger().error("usage: rosrun ik_solver get_tf_ik.py [namespace] [tf_name]")
        exit()

    service_name=sys.argv[1]
    tf_name=sys.argv[2]


    state_pub = node.create_publisher(topic='/ik_solution',msg_type=moveit_msgs.msg.DisplayRobotState, qos_profile=10)
    
    prepend=""
    if(service_name != ""):
        prepend = "/"
    service = prepend+service_name+'/get_ik'

    try:
        ik_locations_srv = node.create_client(srv_name=service, srv_type=ik_solver_msgs.srv.GetIk)
        req = ik_solver_msgs.srv.GetIk.Request()
        req.target.pose.header.frame_id = tf_name
        req.target.pose.pose.position.x = 0.0
        req.target.pose.pose.position.y = 0.0
        req.target.pose.pose.position.z = 0.0
        req.target.pose.pose.orientation.x = 0.0
        req.target.pose.pose.orientation.y = 0.0
        req.target.pose.pose.orientation.z = 0.0
        req.target.pose.pose.orientation.w = 1.0
        future = ik_locations_srv.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        resp = future.result()
        if len(resp.solution.configurations)==0:
            node.get_logger().error("no ik solution")
            exit()

        ik_sol=moveit_msgs.msg.DisplayRobotState()
        ik_sol.state.joint_state.name=resp.joint_names
        ik_sol.state.joint_state.velocity= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ik_sol.state.joint_state.effort= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        idx=0
        node.get_logger().info(f"found {len(resp.solution.configurations)} solution")
        for sol in resp.solution.configurations:
            node.get_logger().debug(f"- {sol.configuration}")

        while True:
            ik_sol.state.joint_state.position=resp.solution.configurations[idx].configuration
            state_pub.publish(ik_sol)
            node.get_clock().sleep_for(Duration(seconds=1, nanoseconds=0))
            idx+=1
            if (idx==len(resp.solution.configurations)):
                idx=0

    except Exception as e:
        print(f"Service call failed: {e}")
