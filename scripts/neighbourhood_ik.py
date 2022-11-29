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

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.

  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.

  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

  return [qx, qy, qz, qw]

def cb(req):
    tf_name=req.tf_name


    service = '/get_ik_array'
    distance=req.distance
    width=req.width
    resolution=req.resolution
    roll=req.roll
    pitch=req.pitch
    yaw=req.yaw

    ik_locations_srv = rospy.ServiceProxy(service, ik_solver_msgs.srv.GetIkArray)
    ik_req = ik_solver_msgs.srv.GetIkArrayRequest()
    poses=geometry_msgs.msg.PoseArray()
    poses.header.frame_id = tf_name
    ik_req.poses.header.frame_id = tf_name
    ik_req.max_number_of_solutions=1
    ik_req.stall_iterations=30

    for distance_z in np.arange(0,distance,resolution):

        width_z=width/distance*distance_z

        for t in np.arange(-width_z*0.5,width_z*0.5,resolution):
            p=geometry_msgs.msg.Pose()
            p.orientation.w=1
            p.position.x=t
            p.position.y=-width_z*0.5
            p.position.z=distance_z
            poses.poses.append(p)

            p=geometry_msgs.msg.Pose()
            p.orientation.w=1
            p.position.x=t
            p.position.y=width_z*0.5
            p.position.z=distance_z
            poses.poses.append(p)

            p=geometry_msgs.msg.Pose()
            p.orientation.w=1
            p.position.x=-width_z*0.5
            p.position.y=t
            p.position.z=distance_z
            poses.poses.append(p)

            p=geometry_msgs.msg.Pose()
            p.orientation.w=1
            p.position.x=width_z*0.5
            p.position.y=t
            p.position.z=distance_z
            poses.poses.append(p)

            p=geometry_msgs.msg.Pose()
            p.orientation.w=1
            p.position.x=t
            p.position.y=t
            p.position.z=distance_z
            poses.poses.append(p)

            p=geometry_msgs.msg.Pose()
            p.orientation.w=1
            p.position.x=-t
            p.position.y=t
            p.position.z=distance_z
            poses.poses.append(p)

    for p in poses.poses:
        ik_req.poses.poses.append(p)

        quat=get_quaternion_from_euler(roll, pitch, yaw)
        p2=geometry_msgs.msg.Pose()
        p2.position=p.position
        p2.orientation.x=quat[0]
        p2.orientation.y=quat[1]
        p2.orientation.z=quat[2]
        p2.orientation.w=quat[3]
        ik_req.poses.poses.append(p2)
        if yaw!=0:
            quat=get_quaternion_from_euler(roll, pitch, -yaw)
            p2=geometry_msgs.msg.Pose()
            p2.position=p.position
            p2.orientation.x=quat[0]
            p2.orientation.y=quat[1]
            p2.orientation.z=quat[2]
            p2.orientation.w=quat[3]
            ik_req.poses.poses.append(p2)

        if pitch!=0:
            quat=get_quaternion_from_euler(roll, -pitch, yaw)
            p2=geometry_msgs.msg.Pose()
            p2.position=p.position
            p2.orientation.x=quat[0]
            p2.orientation.y=quat[1]
            p2.orientation.z=quat[2]
            p2.orientation.w=quat[3]
            ik_req.poses.poses.append(p2)

            if yaw!=0:
                quat=get_quaternion_from_euler(roll, -pitch, -yaw)
                p2=geometry_msgs.msg.Pose()
                p2.position=p.position
                p2.orientation.x=quat[0]
                p2.orientation.y=quat[1]
                p2.orientation.z=quat[2]
                p2.orientation.w=quat[3]
                ik_req.poses.poses.append(p2)

        if roll!=0:
            quat=get_quaternion_from_euler(-roll, pitch, yaw)
            p2=geometry_msgs.msg.Pose()
            p2.position=p.position
            p2.orientation.x=quat[0]
            p2.orientation.y=quat[1]
            p2.orientation.z=quat[2]
            p2.orientation.w=quat[3]
            ik_req.poses.poses.append(p2)
            if yaw!=0:
                quat=get_quaternion_from_euler(-roll, pitch, -yaw)
                p2=geometry_msgs.msg.Pose()
                p2.position=p.position
                p2.orientation.x=quat[0]
                p2.orientation.y=quat[1]
                p2.orientation.z=quat[2]
                p2.orientation.w=quat[3]
                ik_req.poses.poses.append(p2)

            if pitch!=0:
                quat=get_quaternion_from_euler(-roll, -pitch, yaw)
                p2=geometry_msgs.msg.Pose()
                p2.position=p.position
                p2.orientation.x=quat[0]
                p2.orientation.y=quat[1]
                p2.orientation.z=quat[2]
                p2.orientation.w=quat[3]
                ik_req.poses.poses.append(p2)

                if yaw!=0:
                    quat=get_quaternion_from_euler(-roll, -pitch, -yaw)
                    p2=geometry_msgs.msg.Pose()
                    p2.position=p.position
                    p2.orientation.x=quat[0]
                    p2.orientation.y=quat[1]
                    p2.orientation.z=quat[2]
                    p2.orientation.w=quat[3]
                    ik_req.poses.poses.append(p2)

    ik_res = ik_locations_srv(ik_req)
    res=ik_solver_msgs.srv.NeighbourhoodPyramidIkResponse()
    for s in ik_res.solutions:
        print(s)
        if len(s.configurations)==0:
            res.number_of_unreachable_poses=res.number_of_unreachable_poses+1

    res.poses=ik_req.poses
    res.solutions=ik_res.solutions
    res.joint_names=ik_res.joint_names
    return res

def pyramid_server():
    rospy.init_node('pyramid_server')
    s = rospy.Service('~/neighbourhood_ik', ik_solver_msgs.srv.NeighbourhoodPyramidIk, cb)
    rospy.spin()

if __name__ == "__main__":
    pyramid_server()
