#!/usr/bin/env python

# This file contains python script that does object detection in the gazebo world.

# Background color of the world should be a light color and objects should be little bit darker
# for this script to work.

# This code uses the opencv computer vision library, imutils and scipy libraries to detect ad
# find the coordinates of the world objects.

# If you have not installed these libraries the script will not work.

# Author: Hamitcan MALKOC

import cv2
import math
import rospy
import struct
import imutils

import numpy as np
import sensor_msgs.point_cloud2 as pc2

from collections import OrderedDict
from gazebo_msgs.msg import ModelStates
from cv_bridge import CvBridge, CvBridgeError
from ur_detector.msg import ObjectStates, Object
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo

from scipy.spatial import distance as dist

class ShapeDetector:
    def __init__(self):
        pass

    def detect(self, c):
        # detecting the shape of the object
        # we do not necessarily need to detect the shape but we need to find the width
        shape = 'unidentified'
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)

        (x, y), (w, h), theta = cv2.minAreaRect(approx)
        
        if h != 0:
            ar = w / float(h)
        else:
            ar = 1
            w = -1

        if len(approx) == 4:
            shape = 'square' if ar >= 0.95 and ar <= 1.05 else 'rectangle'
        elif len(approx) > 4:
            shape = 'circle'

        return shape, w, theta

class ColorLabeler:
    def __init__(self):
        colors = OrderedDict({
                'black': (0, 0, 0),

                'red': (120, 0, 0),
                'green': (0, 60, 0),
                'blue': (0, 0, 180),

                'white': (255, 255, 255)
            })

        self.lab = np.zeros((len(colors), 1, 3), dtype='uint8')
        self.color_names = []

        for(i, (name, rgb)) in enumerate(colors.items()):
            self.lab[i] = rgb
            self.color_names.append(name)

        self.lab = cv2.cvtColor(self.lab, cv2.COLOR_RGB2LAB)

    def label(self, image, c):
        # labeling the object
        mask = np.zeros(image.shape[ : 2], dtype = 'uint8')
        cv2.drawContours(mask, [c], -1, 255, -1)
        mask = cv2.erode(mask, None, iterations = 2)
        mean = cv2.mean(image, mask = mask)[ : 3]

        # checking every color declared in the __init__ function
        # returning the name of the color with the least euclidean distance
        minDist = (np.inf, None)
        for(i, row) in enumerate(self.lab):
            d = dist.euclidean(row[0], mean)

            if d < minDist[0]:
                minDist = (d, i)

        return self.color_names[minDist[1]]

class ObjectPublisher:
    def __init__(self):
        self.objects = []

        # position of the camera we need to add/subtract these values to find the object's real world position
        # when the class is first initialized these are hard-coded but afterwards we are getting position from /gazebo-model-states
        self.cam_x = 0.900251
        self.cam_y = -0.051019
        self.cam_z = 1.647131

        self.cl = ColorLabeler()
        self.sd = ShapeDetector()

        self.object_pub = rospy.Publisher('objects', ObjectStates, queue_size = 10)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/depth/image_raw", Image, self.callback_image)
        self.point_sub = rospy.Subscriber("camera/depth/points", PointCloud2, self.callback_cloud)
        self.model_sub = rospy.Subscriber('gazebo/model_states', ModelStates, self.callback_model_states)
    
    def callback_image(self, image_data):
        try:
            # there are two different image
            # first 3 channel 8 bit integer array that represents the BGR image of the world
            # sedond 1 channel 32 bit float array that represents the depth image of the world
            # we are working with 8 bit integer array
            if image_data.encoding == 'rgb8':
                # we should not clear self.objects, sometimes it does not work properly when finding the world positions of the objects
                # we needed to create local objects list
                objects = []

                # creating an image to work with opencv
                # resizing the image so that the computation will be littl bit faster
                image = self.bridge.imgmsg_to_cv2(image_data, 'bgr8')
                resized = imutils.resize(image, width = 200)
                ratio = image.shape[0] / float(resized.shape[0])

                # applying blur, converting the image to grayscale image and finding applying the threshold
                # after we apply the threshold object will be represented as white, and the background will be black
                blurred = cv2.GaussianBlur(resized, (3, 3), 0)
                gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
                lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
                thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)[1]
                
                # if you want to see the grayscale and threshold
                # images of the world, comment out these lines (line 143, 144)
                #cv2.imshow('GRAY IMAGE', gray)
                #cv2.imshow('THRESHOLD IMAGE', thresh)

                # finding the contours, with other words objects in the world
                cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cnts = cnts[0] if imutils.is_cv2() else cnts[1]

                # iterating every contour and creating objects with them
                for c in cnts:
                    M = cv2.moments(c)

                    if M['m00'] != 0:
                        # finding the center pixel of the object so that we can find the world position of it
                        # had to multiply it with the ratio to find correct position
                        # ratio is original image size / resized image size
                        # we resized the image to make computation little bit faster
                        cX = int((M['m10'] / M['m00']) * ratio)
                        cY = int((M['m01'] / M['m00']) * ratio)

                        # getting the information about the object (shape, width, rotation, color)
                        # multiplying the width with ratio so we can get the correct width of the object
                        # for now width is in terms of pixels we will convert it into gazebo world unit
                        shape, width, angle = self.sd.detect(c)
                        color = self.cl.label(lab, c)
                        width *= ratio

                        # if you want to see the object in the terminal comment out this line (line 170)
                        #print("{} {} {} {} {}".format(color, shape, cX , cY, width))

                        # if the color of the object is white or black it might be shadow or robot, we are going to skip them
                        # if the width of the object is not in range we are going to skip them
                        if color != 'white' and color != 'black' and width <= 40 and width >= 17:
                            c = c.astype('float')
                            c *= ratio
                            c = c.astype('int')

                            # if you want to see the processed image with contours highlighted,
                            # comment out these lines (line 181, 182, 183, 195, 196)
                            #text = "{} {} {} {} {}".format(color, shape, cX , cY, width)
                            #cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
                            #cv2.putText(image, text, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                            # creating the object and adding to the objects list so that we can find their world position
                            # when we got the point cloud
                            obj = Object(len(objects), 0, 0, 0, cX, cY, width, angle, color, shape)
                            objects.append(obj)
                
                # we have found every object in the world, now we can find their world positions in the point cloud
                # assigning local objects list to class's objects list
                self.objects = objects

                # showing the processed image
                #cv2.imshow('IMAGE', image)
                #cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)
    
    def callback_cloud(self, cloud_data):
        # creating local world objects list we will publish them
        world_objects = []

        # iterating through every object in the class's list and find their actual world position by
        # using point cloud and camera's world position
        for obj in self.objects:
            data_generator = pc2.read_points(cloud_data, ('x', 'y', 'z'), False, [(obj.center_x, obj.center_y)])
            for point in data_generator:
                w_obj = Object(obj.id, self.cam_x + point[0], self.cam_y - point[1], (self.cam_z - point[2]) / 2, obj.center_x, obj.center_y, obj.width, obj.angle, obj.color, obj.shape)
                world_objects.append(w_obj)
                #print('x: {}, y: {}, z: {}, center x: {}, center y: {}'.format(self.cam_x + point[0], self.cam_y - point[1], (self.cam_z - point[2]) / 2, obj.center_x, obj.center_y))
        
        # create correct message type to publish to topic /objects
        objs = ObjectStates(world_objects, len(world_objects))
        self.object_pub.publish(objs)

    def callback_model_states(self, model_data):
        # getting the world position of the camera
        # we did not have any other idea to find the position of the camera
        names = model_data.name
        poses = model_data.pose
        twists = model_data.twist

        i = 0
        while i < len(names):
            if names[i] == 'kinect_ros_1-2':
                break
            i += 1
        
        pose = poses[i]

        self.cam_x = pose.position.x
        self.cam_y = pose.position.y
        self.cam_z = pose.position.z

def explorer_node():
    # create an ObjectPublisher object, it will take care of everything
    op = ObjectPublisher()
    rospy.init_node('object_detector')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('shutting down')

    # when finished close every windows that has been opened while the program is running
    cv2.destroyAllWindows()

if __name__ == '__main__':
    explorer_node()
