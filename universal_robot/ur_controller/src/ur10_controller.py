#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from ur_controller.msg import ObjectStates, Object

def callback(data):
    print "============ New callback data ============"
    moveit_commander.roscpp_initialize(sys.argv)
    print data
    return
    # init moveit_commander
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm_group = moveit_commander.MoveGroupCommander("arm")

    display_trajectroy_publisher = rospy.Publisher('move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20)

    print "============ Waiting for RVIZ ============"
    rospy.sleep(2)
    print "============ Controller starting ============"

    print "============ Printing robot state"
    print robot.get_current_state()

    print "============ Generating plan 1 ============"
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0
    pose_target.position.x = 2
    pose_target.position.y = -0.05
    pose_target.position.z = 1.1
    arm_group.set_pose_target(pose_target)

    plan1 = arm_group.plan()

    print "============ Waiting while RVIZ displays plan1 ============"
    rospy.sleep(5)


    print "Executing plan"
    arm_group.execute(plan1)


def listener():
    rospy.init_node('ur10_controller', anonymous=True)
    rospy.Subscriber("objects", ObjectStates, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInitException:
        pass
