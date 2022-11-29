# collision_checker_server




## Test
run the node _get_tf_ik.py_ providing the name of the IK server and the name of the desired tf. Example:
```
rosrun rosdyn_ik_solver get_tf_ik.py [server_name] [tf_name]
```
The node publish a _moveit_msgs/DisplayRobotState_ topic called _ik_solution_. The robot state shows cyclically the IK solution.
