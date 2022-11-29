#!/usr/bin/env python3
import rospy
import rosservice
import moveit_msgs.msg
import geometry_msgs.msg
import ik_solver_msgs.srv
import actionlib
import random
import time
import sys
import math
import numpy as np

def pyramid_client():
    rospy.init_node('pyramid_client')

    rospy.wait_for_service('/neighbourhood_ik')

    service = rospy.ServiceProxy('/neighbourhood_ik', ik_solver_msgs.srv.NeighbourhoodPyramidIk)

    req=ik_solver_msgs.srv.NeighbourhoodPyramidIkRequest();
    req.width=0.1
    req.distance=0.1
    req.resolution=0.01
    req.roll=0.1
    req.pitch=1
    req.tf_name="foo"
    req.max_number_of_solutions=1
    req.stall_iterations=100

    res=service(req)

    state_pub = rospy.Publisher('/ik_solution',moveit_msgs.msg.DisplayRobotState,queue_size=10)
    array_pub = rospy.Publisher('~/poses',geometry_msgs.msg.PoseArray,queue_size=10)

    ik_sol=moveit_msgs.msg.DisplayRobotState()
    ik_sol.state.joint_state.name=res.joint_names
    ik_sol.state.joint_state.velocity= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ik_sol.state.joint_state.effort= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    r = rospy.Rate(100) # 10hz
    rospy.loginfo("%d poses have no ik solutions",res.number_of_unreachable_poses)
    while not rospy.is_shutdown():
        array_pub.publish(res.poses)
        for sol in res.solutions:
            for conf in sol.configurations:
                ik_sol.state.joint_state.position=conf.configuration
                state_pub.publish(ik_sol)
                r.sleep()
                if (rospy.is_shutdown()):
                    exit()

if __name__ == "__main__":
    pyramid_client()
