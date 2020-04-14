#!/usr/bin/env python

import rospy
import cv2
import imutils
import numpy as np
import pdb
from lab4.msg import cone_location
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
# 
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

class ConeLocator():
    '''
    takes in camera image and returns (x, y) coordinates in meters of cone location relative to the robot
    '''
    ZED_TOPIC = '/zed/rgb/image_rect_color'
    LOCATION_TOPIC = '/relative_location'
    MARKER_TOPIC = '/cone_marker'

    def __init__(self):
        self.bridge = CvBridge()

        # subscribers
        self.zed_sub = rospy.Subscriber(self.ZED_TOPIC, Image, self.callback)
        self.cone_marker_sub = rospy.Subscriber(self.LOCATION_TOPIC, cone_location, self.publish_cone)

        # publishers
        self.loc_pub = rospy.Publisher(self.LOCATION_TOPIC, cone_location, queue_size=10)
        self.cone_marker_pub = rospy.Publisher(self.MARKER_TOPIC, Marker, queue_size=10)
        self.rotated_img_pub = rospy.Publisher('/rotated_image', Image, queue_size=10)

    def callback(self, image):
        '''
        receives zed camera image, converts to ros image
        :param image: zed image
        :return: publishes ros_image to ros_image_topic
        '''
        #ros_image = imutils.rotate(image, 180)
        #self.rotated_img_pub.publish(ros_image)
        try:
            ros_image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        except CvBridgeError as e:
            print(e)

        # rotate ros image 180 degrees
        ros_image = imutils.rotate(ros_image, 180)

        # finding bounding box coords of cone in image
        top_left, bottom_right = self.cd_color_segmentation(ros_image)
        bottom_center = np.array([(top_left[0]+bottom_right[0])/2, bottom_right[1], 1]) # coordinate of bottom center of the cone bounding box (front of cone)

        # translating image coordinates to x, y distances in meters of real world

        # homography matrix

        homo_mat = np.array([[ 1.10578622e-04,  8.35751019e-04,  7.72309592e-02 ],
                   [ 5.84026363e-04,  5.96178710e-05, -2.02175859e-01 ],
                   [-2.17320149e-04, -6.35124861e-03,  1.00000000e+00]])

        bottom_center[0] = 672-bottom_center[0]
        bottom_center[1] = 376-bottom_center[1]
        bottom_center = bottom_center.T # transpose point for matrix mul
        x_y_meters = np.matmul(homo_mat, bottom_center)
        x_y_meters = x_y_meters/x_y_meters[2]

        # distance of robot from cone
        x = x_y_meters[0] # in meters
        y = x_y_meters[1] # origin offset by 11 cm

        # publish location onto relative cone location topic
        loc = cone_location()
        loc.x_pos = x
        loc.y_pos = y
        self.loc_pub.publish(loc)

        cv_rot_im = cv2.rectangle(ros_image, top_left, bottom_right, (255,0,0), 2)
        bc = bottom_center.T
        cv_rot_im = cv2.rectangle(ros_image, (bc[0], bc[1]), (bc[0]+5, bc[1]+5), (0, 255, 0), 5)
        cv_rot_im = self.bridge.cv2_to_imgmsg(ros_image)
        self.rotated_img_pub.publish(cv_rot_im)

        rospy.loginfo(loc)

    def cd_color_segmentation(self, img):
        '''
        finds the coordinates of the bounding box for the orange cone in the image
        :param img: ros image with cone
        :return: top left and bottom right coordinates of bounding box
        '''
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # convert image to HSV color space

        # define range of orange color in HSV
        upper_orange = np.array([30, 255, 255])
        lower_orange = np.array([3, 190, 190])

        # Threshold the HSV iamge to get only orange colors
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Erosion and Dilation
        kernel = np.ones((6, 6), np.uint8)
        ero_dil = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # finding bounding rectangle for cone using erosion/dilation mask
        x, y, w, h = cv2.boundingRect(ero_dil)
        bounding_box = ((x, y), (x+w, y+h))

        # drawing the bounding box onto the image
        #cv2.rectangle(img, bounding_box[0], bounding_box[1], (255,0,0), 2)

        # Return bounding box
        return bounding_box

    def publish_cone(self, cone_loc):
        '''
        publish cone as marker in rviz
        '''
        marker = Marker()
        marker.id = 0
        marker.type = Marker.POINTS
        marker.header.frame_id = 'base_link'
        marker.header.stamp = rospy.get_rostime()
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.pose.orientation.w = 1

        marker.pose.position.x = cone_loc.x_pos
        marker.pose.position.y = cone_loc.y_pos

        self.cone_marker_pub.publish(marker)

if __name__  == '__main__':
    rospy.init_node('cone_location')
    ConeLocator()
    rospy.spin()
