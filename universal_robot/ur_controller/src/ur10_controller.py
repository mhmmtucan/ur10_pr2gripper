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
    # init moveit_commander
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm_group = moveit_commander.MoveGroupCommander("arm")

    display_trajectroy_publisher = rospy.Publisher('move_group/display_planned_path',
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20)
    objectToGrip = Object()
    # taking objects, getting rid of unknown objecs
    for x in data.objects:
        if x.color == "blue":
            # this is object to grip
            objectToGrip = x
            break
    
    print "============ Waiting for RVIZ ============"
    #rospy.sleep(2)

    print "============ Controller starting ============"
    print objectToGrip	
    #print "============ Printing robot state"
    #print robot.get_current_state()

    print "============ Generating plan ============"
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = objectToGrip.angle
    pose_target.position.x = objectToGrip.position_x
    pose_target.position.y = objectToGrip.position_y
    pose_target.position.z = objectToGrip.position_z
    arm_group.set_pose_target(pose_target)

    plan1 = arm_group.plan()

    #print "============ Waiting while RVIZ displays plan1 ============"
    #rospy.sleep(5)


    print "============ Executing plan ============"
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
