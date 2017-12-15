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
#gripper_group = MoveGroupCommander("gripper")
rospy.sleep(1)
object_dict = {}

def callback(data):
    print "============ New callback data"
    waypoints = []
    global arm_group, object_dict, robot
    
    object_to_grip = Object()
                
    if len(data.objects) != 0:
        # we have objects, go on
        for a in data.objects:
            # take all objects and add to scene and object dict
            if str(a.object_id) not in object_dict or object_dict[str(a.object_id)] != "placed":
                object_dict[str(a.object_id)] = a
                
                # add objects to world
                addToScene(pos=(a.position_x,a.position_y,a.position_z),
                    w=1,size=(0.075, 0.075, 0.180),name="object"+str(a.object_id))
                    
                object_to_grip = a
                
    print object_to_grip
    
    print "============ Printing robot state"
    print robot.get_current_state()
    
    object_dict[str(object_to_grip.object_id)] = "placed"
    
    print "============ Waiting while RVIZ displays plan"
    rospy.sleep(2)
                             
    print "============ Executing plan"
    # TODO: check whether object id is None or not
    robot.arm.pick("object"+str(object_to_grip.object_id))
    robot.arm.place(object_to_grip.color+"_zone")
    
    
def init():
    roscpp_initialize(sys.argv)
    global scene,robot

    # clean the scene
    cleanWorld()
    
    # save base pose in order to use later
    #arm_group.remember_joint_values("base_pose")
    
    # display planned trajectory to rviz in order to visualize
    display_trajectroy_publisher = rospy.Publisher('move_group/display_planned_path',
        DisplayTrajectory,
        queue_size=20)

    createWorld()
 
 
def createWorld():
    # add cylindral platform to scene
    addToScene(pos=(0,0,0.339),w=1,size=(0.2, 0.2, 0.90),name="platform")
    
    # add ground in order to prevent ground-collision
    addToScene(pos=(0,0,-0.005),w=1,size=(3, 3, 0.01),name="ground")
    
    addToScene(pos=(0.9,-0.051,1.647),orient=
        Quaternion(*tf_conversions.transformations.quaternion_from_euler(-1.507854, 1.545821, 0.060861)),
        size=(0.073, 0.276, 0.072),name="kinect")
        
    # add containers
        
        
def cleanWorld():
    deleteFromScene("ground")
    deleteFromScene("platform")
    deleteFromScene("kinect")
    
    
def addToScene(pos,orient,w,size,name):
    global scene, robot
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = pos[0]
    p.pose.position.x = pos[1]
    p.pose.position.x = pos[2]
    if w:
        p.pose.orientation.w = w
    else:
        p.pose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(orient[0], orient[1], orient[2]))
    scene.add_box(name,p,size)
    
        
def deleteFromScene(name):
    global scene
    scene.remove_world_object(name)
    

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

