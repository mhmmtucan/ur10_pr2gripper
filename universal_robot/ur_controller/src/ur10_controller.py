#!/usr/bin/env python
import sys
import rospy
import copy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, DisplayTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ur_controller.msg import ObjectStates, Object

def callback(data):
    print "============ New callback data"
    roscpp_initialize(sys.argv)
    waypoints = []
    
    # init moveit_commander
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    arm_group = MoveGroupCommander("arm")
    #gripper_group = MoveGroupCommander("gripper")
    rospy.sleep(1)
    
    # clean scene
    scene.remove_world_object("platform")
    
    # save base pose in order to use later
    #arm_group.remember_joint_values("base_pose")
    
    # display planned trajectory to rviz in order to visualize
    display_trajectroy_publisher = rospy.Publisher('move_group/display_planned_path',
        DisplayTrajectory,
        queue_size=20)
        
    # add cylindral platform to scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0
    p.pose.position.y = 0
    p.pose.position.z = 0.339
    p.pose.orientation.w = 1.0
    scene.add_box("platform", p, (0.2, 0.2, 0.90))
    
    # if there is incoming data, and object is object to grasp, take it
    object_to_grip = Object()
    if len(data.objects) != 0:
        for x in data.objects:
            if (x.color != "white" or x.color != "black") and x.shape != "circle":
                object_to_grip = x
                break
    
    print object_to_grip
    
    print "============ Printing robot state"
    print robot.get_current_state()
    
    print "============ Generating plan ============"
    #target_object_pose = Pose()
    #target_object_pose.orientation.w = object_to_grip.angle
    #target_object_pose.position.x = object_to_grip.position_x
    #target_object_pose.position.y = object_to_grip.position_y
    #target_object_pose.position.z = object_to_grip.position_z
    
    # go to object
    waypoints.append(copy.deepcopy(target_object_pose))
    
    # pickup
    
    
    # go to dropzone
    dropzone_pose = dropzone(object_to_grip.color)
    waypoints.append(copy.deepcopy(dropzone_pose))
    
    # place
    
    
    (plan, fraction) = arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    
    print "============ Waiting while RVIZ displays plan3..."
    rospy.sleep(2)
                             
    print "============ Executing plan ============"
    arm_group.execute(plan)
    
    
def dropzone(color):
    dropzone_pose = Pose()
    if color == "blue":
        dropzone_pose.orientation.w = 0.946450
        dropzone_pose.position.x = -0.368809
        dropzone_pose.position.y = -0.882567
        dropzone_pose.position.z = 0.2
    elif color == "green":
        dropzone_pose.orientation.w = 0.946450
        dropzone_pose.position.x = -0.685113
        dropzone_pose.position.y = -0.001002
        dropzone_pose.position.z = 0.2
    elif color == "red":
        dropzone_pose.orientation.w = 0.656062
        dropzone_pose.position.x = -0.300907
        dropzone_pose.position.y = 0.820711
        dropzone_pose.position.z = 0.2
    return dropzone_pose

def listener():
    rospy.init_node('ur10_controller', anonymous=True)
    rospy.Subscriber("objects", ObjectStates, callback)
    rospy.spin()
    roscpp_shutdown()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInitException:
        pass
