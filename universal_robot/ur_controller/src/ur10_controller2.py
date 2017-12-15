#!/usr/bin/env python

# This controller file for ur10_arm with gripper, but if it is the case
# that gripper is not working, rather than picking and placing objects
# after mimicing grasping dropping, tag objects picked and placed.
# do not use moveit pick and place functions because of collisions.

# Author: Muhammet Ucan

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

# init moveit_commander
robot = RobotCommander()
scene = PlanningSceneInterface()
arm_group = MoveGroupCommander("arm")
gripper_group = MoveGroupCommander("gripper")
rospy.sleep(1)
object_dict = {}

def callback(data):
    print "============ New callback data"
    waypoints = []
    global arm_group, object_dict, scene
    
    print data.objects
    num = 0
    object_to_grip = Object() 
    scene_objects = scene.get_objects().keys()
    if "paltform" not in scene_objects and "kinetic" not in scene.get_objects().keys() and "ground" not in scene.get_objects().keys():
            createWorld()
            print scene.get_objects().keys()
            
    print "robot.get_joint_names()"  
    print robot.get_joint_names()
    print "robot.get_link_names()"
    print robot.get_link_names()
    print "robot.get_planning_frame()"
    print robot.get_planning_frame()
    print "gripper_group.get_active_joints()"
    print gripper_group.get_active_joints()    
    if len(data.objects) != 0:
        # we have objects, go on
        for a in data.objects:
            # take all objects and add to scene and object dict
            if str(num) not in object_dict or object_dict[str(num)] != "placed":
                object_dict[str(num)] = a
                
                # add objects to world
                addToScene(pos=(a.position_x,a.position_y,a.position_z),orient='',
                    w=1,size=(0.075, 0.075, 0.180),name="object"+str(num))
                    
                object_to_grip = a
                num+=1

    
def init():
    roscpp_initialize(sys.argv)    
    createWorld()
    # save base pose in order to use later
    #arm_group.remember_joint_values("base_pose")
    
    # display planned trajectory to rviz in order to visualize
    display_trajectroy_publisher = rospy.Publisher('move_group/display_planned_path',
        DisplayTrajectory,
        queue_size=20)
        
 
def createWorld():
    global scene
    # add cylindral platform to scene
    addToScene(pos=(0,0,0.339),orient='',w=1,size=(0.2, 0.2, 0.90),name="platform")
    
    # add ground in order to prevent ground-col
    addToScene(pos=(0,0,-0.005),orient='',w=1,size=(3, 3, 0.01),name="ground")
    
    addToScene(pos=(0.9,-0.051,1.647),orient=(-1.507854, 1.545821, 0.060861),
        w='',size=(0.073, 0.276, 0.072),name="kinect")
      
    addToScene(pos=(-0.368809,-0.882567,0),orient=(0,0,0.946450),
        w='',size=(0.6, 0.6, 0.02),name="blue_target")
    addToScene(pos=(-0.685113,0.001002,0),orient=(0,0,0),
        w='',size=(0.6, 0.6, 0.02),name="green_target")
    addToScene(pos=(-0.366010,0.878589,0),orient=(0,0,0.656062),
        w='',size=(0.6, 0.6, 0.02),name="red_target")
    
    
def addToScene(pos,orient,w,size,name):
    global scene, robot
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()

    p.pose.position.x = pos[0]
    p.pose.position.y = pos[1]
    p.pose.position.z = pos[2]
    if w:
        p.pose.orientation.w = w
    else:
        p.pose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(orient[0], orient[1], orient[2]))
    scene.add_box(name,p,size)
    
        
def deleteFromScene(name):
    global scene
    scene.remove_world_object(name)
    print name + "removing"
    
    
def dropzone(color):
    dropzone_pose = Pose()
    if color == "blue":
        dropzone_pose.orientation.w = 0.946450
        dropzone_pose.position.x = -0.368809
        dropzone_pose.position.y = -0.882567
        dropzone_pose.position.z = 0.3
    elif color == "green":
        dropzone_pose.orientation.w = 0.946450
        dropzone_pose.position.x = -0.685113
        dropzone_pose.position.y = -0.001002
        dropzone_pose.position.z = 0.3
    elif color == "red":
        dropzone_pose.orientation.w = 0.656062
        dropzone_pose.position.x = -0.300907
        dropzone_pose.position.y = 0.820711
        dropzone_pose.position.z = 0.3
    return dropzone_pose


def listener():
    rospy.init_node('ur10_controller', anonymous=True)
    init()
    rospy.Subscriber("objects", ObjectStates, callback)
    
    rospy.spin()
    
    roscpp_shutdown()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInitException:
        pass
