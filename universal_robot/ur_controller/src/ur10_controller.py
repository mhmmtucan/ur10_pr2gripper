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

# init moveit_commander
robot = RobotCommander()
scene = PlanningSceneInterface()
arm_group = MoveGroupCommander("arm")
#gripper_group = MoveGroupCommander("gripper")
rospy.sleep(1)

def callback(data):
    print "============ New callback data"
    waypoints = []
    global arm_group,scene,robot
    
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.orientation.w = 1.0
    
    # if there is incoming data, and object is object to grasp, take it
    object_to_grip = Object()
    if len(data.objects) != 0:
        for a in data.objects:
            if (a.color != "white" or a.color != "black") and a.shape != "circle":
                object_to_grip = a
                
                # add all objects to scene
                p.pose.position.x = a.position_x
                p.pose.position.y = a.position_y
                p.pose.position.z = a.position_z
                
                # MARK: generate names and add to scene
                
                
                
    
    print object_to_grip
    
    print "============ Printing robot state"
    print robot.get_current_state()
    
    print "============ Generating plan"
    target_object_pose = Pose()
    target_object_pose.orientation.w = object_to_grip.angle
    target_object_pose.position.x = object_to_grip.position_x
    target_object_pose.position.y = object_to_grip.position_y
    target_object_pose.position.z = object_to_grip.position_z
    
    # go to object
    waypoints.append(copy.deepcopy(target_object_pose))
    
    # pickup
    
    
    # go to dropzone
    dropzone_pose = dropzone(object_to_grip.color)
    waypoints.append(copy.deepcopy(dropzone_pose))
    
    # place
    
    
    (plan, fraction) = arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    
    print "============ Waiting while RVIZ displays plan"
    rospy.sleep(2)
                             
    print "============ Executing plan"
    arm_group.execute(plan)
    
    
def init():
    roscpp_initialize(sys.argv)
    global scene,robot

    # clean the scene
    scene.remove_world_object("ground")
    scene.remove_world_object("platform")
    scene.remove_world_object("kinect")
    
    # save base pose in order to use later
    #arm_group.remember_joint_values("base_pose")
    
    # display planned trajectory to rviz in order to visualize
    display_trajectroy_publisher = rospy.Publisher('move_group/display_planned_path',
        DisplayTrajectory,
        queue_size=20)
            
    # add cylindral platform to scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.orientation.w = 1.0
    p.pose.position.x = 0
    p.pose.position.y = 0
    p.pose.position.z = 0.339
    scene.add_box("platform", p, (0.2, 0.2, 0.90))
    
    # add ground in order to prevent ground-col
    p.pose.position.z = -0.005
    scene.add_box("ground",p, (3, 3, 0.01))
    
    # add kinect
    p.pose.position.x = 0.9
    p.pose.position.y = -0.051
    p.pose.position.z = 1.647
    p.pose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(-1.507854, 1.545821, 0.060861))
    scene.add_box("kinect", p, (0.073, 0.276, 0.072))
    
    
def addToScene():
    pass
    
    
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
    init()
    rospy.Subscriber("objects", ObjectStates, callback)
    rospy.spin()
    roscpp_shutdown()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInitException:
        pass
