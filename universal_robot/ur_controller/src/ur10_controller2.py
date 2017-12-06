#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from ur_controller.msg import ObjectStates, Object


rospy.init_node('ur10_controller', anonymous=True)

moveit_commander.roscpp_initialize(sys.argv)

# init moveit_commander
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")

display_trajectroy_publisher = rospy.Publisher('move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)


print "============ Waiting for RVIZ ============"
rospy.sleep(2)
	
#print "============ Printing robot state"
#print robot.get_current_state()

print "============ Generating plan ============"
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = -0.690611
pose_target.position.y = -0.700840
pose_target.position.z = 0.2
group.set_pose_target(pose_target)


plan1 = group.plan()

print "============ Waiting while RVIZ displays plan1 ============"
rospy.sleep(3)


print "============ Executing plan ============"
group.execute(plan1)
rospy.spin()
moveit_commander.roscpp_shutdown()
