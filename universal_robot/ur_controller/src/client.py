#!/usr/bin/env python
import roslib
roslib.load_manifest('ur_controller')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal,GripperCommandAction


rospy.init_node('joint_position_tester')
client = actionlib.SimpleActionClient('/gripper_controller/gripper_cmd', GripperCommandAction)
#client.wait_for_server():
rospy.loginfo("Waiting for gripper action server")
if client.wait_for_server(rospy.Duration(1)):
    rospy.logwarn("Found gripper action server")
else:
    rospy.logwarn("Cannot find gripper action server")
