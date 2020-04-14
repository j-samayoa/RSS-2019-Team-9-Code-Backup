#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

import math

SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
VELOCITY = rospy.get_param("wall_follower/velocity")

pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)

def callback(data):
    '''
    always drive straight
    '''
    ranges = data.ranges

    #slice_range = ranges[ati(data, math.pi/3)-5:ati(data,math.pi/3)+5] # front
    slice_range = ranges[ati(data, 5*math.pi/6)-5:ati(data, 5*math.pi/6)+5] # left
    dis = sum(slice_range)/len(slice_range)
    print(dis)
    print(dis == float('Inf'))


    drive_data = AckermannDriveStamped()
    drive_data.drive.speed = VELOCITY
    pub.publish(drive_data)

def ati(data, angle):
    return int((angle-data.angle_min)/data.angle_increment)

sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, callback)

if __name__ == "__main__":
    rospy.init_node('straight')
    rospy.spin()
