#!/usr/bin/env python3
import rospy
import rosservice
import moveit_msgs.msg
import ik_solver_msgs.srv
import actionlib
import random
import time
import sys


if __name__ == "__main__":
    rospy.init_node('get_ik')


    if len(sys.argv)<3:
        rospy.logerr("usage: rosrun ik_solver get_tf_ik.py [namespace] [tf_name]")
        exit()

    service_name=sys.argv[1]
    tf_name=sys.argv[2]


    state_pub = rospy.Publisher('/ik_solution',moveit_msgs.msg.DisplayRobotState,queue_size=10)
    service = '/'+service_name+'/get_ik'

    try:
        ik_locations_srv = rospy.ServiceProxy(service, ik_solver_msgs.srv.GetIk)
        req = ik_solver_msgs.srv.GetIkRequest()
        req.target.pose.header.frame_id = tf_name
        req.target.pose.pose.position.x = 0.0
        req.target.pose.pose.position.y = 0.0
        req.target.pose.pose.position.z = 0.0
        req.target.pose.pose.orientation.x = 0.0
        req.target.pose.pose.orientation.y = 0.0
        req.target.pose.pose.orientation.z = 0.0
        req.target.pose.pose.orientation.w = 1.0
        resp = ik_locations_srv(req)
        if len(resp.solution.configurations)==0:
            rospy.logerr("no ik solution")
            exit()

        ik_sol=moveit_msgs.msg.DisplayRobotState()
        ik_sol.state.joint_state.name=resp.joint_names
        ik_sol.state.joint_state.velocity= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ik_sol.state.joint_state.effort= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        idx=0
        rospy.loginfo("found %d solution",len(resp.solution.configurations))
        for sol in resp.solution.configurations:
            rospy.logdebug("- ",sol.configuration)

        r = rospy.Rate(len(resp.solution.configurations)/10) # all solution in 10 seconds
        while not rospy.is_shutdown():
            ik_sol.state.joint_state.position=resp.solution.configurations[idx].configuration
            state_pub.publish(ik_sol)
            r.sleep()
            idx+=1
            if (idx==len(resp.solution.configurations)):
                idx=0

    except rospy.ServiceException as e:
        print("[GET TF IK] Service call failed: %s"%e)
