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

if __name__ == "__main__":
    rospy.init_node('get_ik')


    if len(sys.argv)<3:
        rospy.logerr("usage: rosrun rosdyn_ik_solver [namespace] [tf_name]")
        exit()

    service_name=sys.argv[1]
    tf_name=sys.argv[2]


    state_pub = rospy.Publisher('/ik_solution',moveit_msgs.msg.DisplayRobotState,queue_size=10)
    array_pub = rospy.Publisher('/poses',geometry_msgs.msg.PoseArray,queue_size=10)
    service = '/'+service_name+'/get_ik_array'
    r = rospy.Rate(500) # 10hz
    try:
        ik_locations_srv = rospy.ServiceProxy(service, ik_solver_msgs.srv.GetIkArray)
        req = ik_solver_msgs.srv.GetIkArrayRequest()
        req.poses.header.frame_id = tf_name
        req.max_number_of_solutions=32
        req.stall_iterations=30
        for t in np.arange(0,2,0.005):
            p=geometry_msgs.msg.Pose()
            p.orientation.w=1
            p.position.x=0.15*math.sin(2*math.pi*t)+0.05*math.sin(2*math.pi*t*10)
            p.position.y=                           0.05*math.cos(2*math.pi*t*10)
            p.position.z=0.15*math.cos(2*math.pi*t)
            req.poses.poses.append(p)

        for ip in range(0,100):
            array_pub.publish(req.poses)
            #rospy.loginfo(req.poses)
            r.sleep()

        resp = ik_locations_srv(req)
        if len(resp.solutions)==0:
            rospy.logerr("no ik solution")
            exit()

        ik_sol=moveit_msgs.msg.DisplayRobotState()
        ik_sol.state.joint_state.name=resp.joint_names
        ik_sol.state.joint_state.velocity= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ik_sol.state.joint_state.effort= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        idx=0
        while not rospy.is_shutdown():
            array_pub.publish(req.poses)
            for sol in resp.solutions:
                for conf in sol.configurations:
                    ik_sol.state.joint_state.position=conf.configuration
                    state_pub.publish(ik_sol)
                    r.sleep()
                    if (rospy.is_shutdown()):
                        exit()

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
