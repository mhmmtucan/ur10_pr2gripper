#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from ur_controller.msg import ObjectStates, Object


rospy.init_node('ur10_controller', anonymous=True)

print "============ New callback data ============"
moveit_commander.roscpp_initialize(sys.argv)

# init moveit_commander
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")

display_trajectroy_publisher = rospy.Publisher('move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

# taking objects, getting rid of unknown objecs

print "============ Waiting for RVIZ ============"
rospy.sleep(2)
	
#print "============ Printing robot state"
#print robot.get_current_state()

print "============ Generating plan ============"
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = 0.7
pose_target.position.y = -0.05
pose_target.position.z = 1.1
group.set_pose_target(pose_target)


plan1 = group.plan()

print "============ Waiting while RVIZ displays plan1 ============"
rospy.sleep(5)


print "============ Executing plan ============"
group.execute(plan1)
rospy.spin()