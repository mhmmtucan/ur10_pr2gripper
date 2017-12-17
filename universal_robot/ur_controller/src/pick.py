#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Ioan Sucan

import sys
import rospy
import copy
import tf_conversions
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ur_controller.msg import ObjectStates, Object

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    rospy.sleep(1)

    # clean the scene
    scene.remove_world_object("box1")
    scene.remove_world_object("ground")
    scene.remove_world_object("platform")
    scene.remove_world_object("cam")
    scene.remove_world_object("red")

    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.orientation.w = 1.0
    p.pose.position.x = 0.767938
    p.pose.position.y = -0.213985
    p.pose.position.z = 0.060 + 0.2
    scene.add_box("box1", p, (0.075, 0.075, 0.180))
    
    # add cylindral platform to scene
    p.pose.position.x = 0
    p.pose.position.y = 0
    p.pose.position.z = 0.339
    scene.add_box("platform", p, (0.2, 0.2, 0.90))
    
    p.pose.position.z = -0.005
    scene.add_box("ground",p, (3, 3, 0.01))
    
    p.pose.position.x = 0.9
    p.pose.position.y = -0.051
    p.pose.position.z = 1.647
    p.pose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(-1.507854, 1.545821, 0.060861))
    scene.add_box("cam", p, (0.073, 0.276, 0.072))
    
    
    rospy.sleep(1)

    # pick an object

    arm_group = MoveGroupCommander("arm")
 
    p.pose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, 0.946450))
    p.pose.position.x = -0.368809
    p.pose.position.y = -0.882567
    p.pose.position.z = 0.3
    scene.add_box("blue",p,(0.6, 0.6, 0.02))
    
    dropzone_pose = Pose()
    dropzone_pose2 = Pose()
    dropzone_pose3 = Pose()
    
    dropzone_pose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, 0.0))
    dropzone_pose.position.x = -0.717746
    dropzone_pose.position.y = -0.458615
    dropzone_pose.position.z = 0.25

    dropzone_pose2.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, 0.0))
    dropzone_pose2.position.x = -0.649493
    dropzone_pose2.position.y = 0.742142
    dropzone_pose2.position.z = 0.25
    
    dropzone_pose3.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, 0.0))
    dropzone_pose3.position.x = -0.871420
    dropzone_pose3.position.y = 0.142003
    dropzone_pose3.position.z = 0.25
    
    arm_group.set_pose_target(dropzone_pose2)
    plan_arm = arm_group.plan()
    print "Waiting for Rviz to show plan1"
    rospy.sleep(3)
    print "Executing-----"
    arm_group.execute(plan_arm)
    

    
    #p = Pose()
    #p.orientation.w = 1
    #p.position.x = 0.7
    #p.position.y = -0.05
    #p.position.z = 1.1
    #arm_group.set_pose_target(p)
    
    plan_arm = arm_group.plan()

    
    
    

    
    #print "Executing plan"
    #robot.arm.pick("box1")
    
    
    #gripper_group = MoveGroupCommander("gripper")
    #print "Active joints",gripper_group.get_active_joints()
    #gripper_group.set_named_target("open")
    #plan1 = gripper_group.plan()
    #gripper_group.execute(plan1)
    
    #group_variable_values = gripper_group.get_current_joint_values()
    #print "============ Joint values: ", group_variable_values
	
	
    #group_variable_values[0] = 0.5
    #gripper_group.set_joint_value_target(group_variable_values)
    #print "============ Joint values after: ", group_variable_values
    #plan2 = gripper_group.plan()
    #gripper_group.execute(plan2)
    rospy.spin()
    roscpp_shutdown()
