#!/usr/bin/env python

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
from robotics_wp1.msg import ObjectStates, Object
from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo

from scipy.spatial import distance as dist

class ShapeDetector:
    def __init__(self):
        pass

    def detect(self, c):
        shape = 'unidentified'
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)

        (x, y), (w, h), theta = cv2.minAreaRect(approx)
        ar = w / float(h)

        if len(approx) == 4:
            shape = 'square' if ar >= 0.95 and ar <= 1.05 else 'rectangle'
        else:
            shape = 'circle'

        return shape, w, theta

class ColorLabeler:
    def __init__(self):
        colors = OrderedDict({
                'black': (0, 0, 0),

                'dark red': (50, 0, 0),
                'dark green': (0, 50, 0),
                'dark blue': (0, 0, 50),

                'red': (150, 0, 0),
                'green': (0, 150, 0),
                'blue': (0, 0, 150),

                'light red': (255, 0, 0),
                'light green': (0, 255, 0),
                'light blue': (0, 0, 255),

                'white': (255, 255, 255)
            })

        self.lab = np.zeros((len(colors), 1, 3), dtype='uint8')
        self.color_names = []

        for(i, (name, rgb)) in enumerate(colors.items()):
            self.lab[i] = rgb
            self.color_names.append(name)

        self.lab = cv2.cvtColor(self.lab, cv2.COLOR_RGB2LAB)

    def label(self, image, c):
        mask = np.zeros(image.shape[ : 2], dtype = 'uint8')
        cv2.drawContours(mask, [c], -1, 255, -1)
        mask = cv2.erode(mask, None, iterations = 2)
        mean = cv2.mean(image, mask = mask)[ : 3]

        minDist = (np.inf, None)

        for(i, row) in enumerate(self.lab):
            d = dist.euclidean(row[0], mean)

            if d < minDist[0]:
                minDist = (d, i)

        return self.color_names[minDist[1]]

class ObjectPublisher:
    def __init__(self):
        self.objects = []

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
            if image_data.encoding == 'rgb8':
                self.objects = []
                
                image = self.bridge.imgmsg_to_cv2(image_data, 'bgr8')
                resized = imutils.resize(image, width = 200)
                ratio = image.shape[0] / float(resized.shape[0])

                blurred = cv2.GaussianBlur(resized, (3, 3), 0)
                gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
                lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
                thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)[1]
                
                #cv2.imshow('GRAY IMAGE', gray)
                #cv2.imshow('THRESHOLD IMAGE', thresh)

                edges = cv2.Canny(image, 100, 200)
                cv2.imshow('edges', edges)
                cv2.waitKey(1)

                cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cnts = cnts[0] if imutils.is_cv2() else cnts[1]

                for c in cnts:
                    M = cv2.moments(c)

                    if M['m00'] != 0:
                        cX = int((M['m10'] / M['m00']) * ratio)
                        cY = int((M['m01'] / M['m00']) * ratio)

                        shape, width, angle = self.sd.detect(c)
                        color = self.cl.label(lab, c)

                        #if color != 'white' and color != 'black' and width <= 20 and width >= 10:
                        c = c.astype('float')
                        c *= ratio
                        c = c.astype('int')

                        width *= ratio

                        text = "{} {} {} {} {}".format(color, shape, cX , cY, width)

                        #cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
                        #cv2.putText(image, text, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        obj = Object(0, 0, 0, cX, cY, width, angle, color, shape)
                        self.objects.append(obj)

                #cv2.imshow('IMAGE', image)
                #cv2.waitKey(1)
            
        except CvBridgeError as e:
            print(e)
    
    def callback_cloud(self, cloud_data):
        world_objects = []

        for obj in self.objects:
            data_generator = pc2.read_points(cloud_data, ('x', 'y', 'z'), False, [(obj.center_x, obj.center_y)])
            for point in data_generator:
                w_obj = Object(self.cam_x + point[0], self.cam_y - point[1], (self.cam_z - point[2]) / 2, obj.center_x, obj.center_y, obj.width, obj.angle, obj.color, obj.shape)
                world_objects.append(w_obj)
                #print('x: {}, y: {}, z: {}, center x: {}, center y: {}'.format(self.cam_x + point[0], self.cam_y - point[1], (self.cam_z - point[2]) / 2, obj.center_x, obj.center_y))
        
        objs = ObjectStates(world_objects, len(world_objects))

        self.object_pub.publish(objs)

    def callback_model_states(self, model_data):
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
    op = ObjectPublisher()
    rospy.init_node('object_detector')

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('shutting down')

    cv2.destroyAllWindows()

if __name__ == '__main__':
    explorer_node()