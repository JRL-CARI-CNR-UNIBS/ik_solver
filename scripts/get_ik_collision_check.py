#!/usr/bin/env python3
import rospy
import rosservice
import moveit_msgs.msg
import moveit_msgs.srv

import ik_solver_msgs.srv
import std_msgs.msg
import actionlib
import random
import time
import sys


if __name__ == "__main__":
    rospy.init_node('get_ik_collision')


    if len(sys.argv)<3:
        rospy.logerr("usage: rosrun rosdyn_ik_solver [namespace] [tf_name]")
        exit()

    service_name=sys.argv[1]
    tf_name=sys.argv[2]

    get_scene = rospy.ServiceProxy('/get_planning_scene', moveit_msgs.srv.GetPlanningScene)
    scene_req=moveit_msgs.srv.GetPlanningSceneRequest()
    scene_res = get_scene(scene_req)
    scene_req.components.components=1023 # all info
    print(scene_res)
    state_pub = rospy.Publisher('/ik_solution',moveit_msgs.msg.DisplayRobotState,queue_size=10)
    r = rospy.Rate(10) # 10hz
    try:
        ik_locations_srv = rospy.ServiceProxy('/'+service_name+'/get_ik', ik_solver_msgs.srv.GetIk)
        req = ik_solver_msgs.srv.GetIkRequest()
        req.tf_name = tf_name
        resp = ik_locations_srv(req)
        if len(resp.solution.configurations)==0:
            rospy.logerr("no ik solution")
            exit()

        cc_srv = rospy.ServiceProxy('/ik_collision_check/collision_check', ik_solver_msgs.srv.CollisionChecking)
        cc_req = ik_solver_msgs.srv.CollisionCheckingRequest()
        cc_req.joint_names = resp.joint_names
        cc_req.solutions=resp.solution.configurations
        cc_req.detailed=True
        cc_req.planning_scene=scene_res.scene

        cc_resp = cc_srv(cc_req)
        print(cc_resp)


        idx=0
        rgba=std_msgs.msg.ColorRGBA()
        rgba.a=1
        rgba.r=1
        while not rospy.is_shutdown():
            ik_sol=moveit_msgs.msg.DisplayRobotState()
            ik_sol.state.joint_state.name=resp.joint_names
            ik_sol.state.joint_state.velocity= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            ik_sol.state.joint_state.effort= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            ik_sol.state.joint_state.position=cc_resp.result[idx].solution.configuration

            for id in cc_resp.result[idx].colliding_obj_first:
                oc=moveit_msgs.msg.ObjectColor()
                oc.color=rgba
                oc.id=id
                ik_sol.highlight_links.append(oc)
            for id in cc_resp.result[idx].colliding_obj_second:
                oc=moveit_msgs.msg.ObjectColor()
                oc.color=rgba
                oc.id=id
                ik_sol.highlight_links.append(oc)
            if (not cc_resp.result[idx].out_of_bound):
                state_pub.publish(ik_sol)
            r.sleep()
            idx+=1

            if (idx==len(resp.solution.configurations)):
                idx=0

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
