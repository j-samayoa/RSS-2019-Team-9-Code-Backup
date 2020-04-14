#!/usr/bin/env python
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
class ScanShifter(object):
    def __init__(self):
        rospy.init_node("scan_shifter")
        self.sub = rospy.Subscriber("/scan",LaserScan,self.shift_scan)
        self.pub = rospy.Publisher("/shifted_scan",LaserScan,queue_size=10)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            rate.sleep()
    def shift_scan(self,msg):
        prc = 0.17
        ranges = msg.ranges
        intensities = msg.intensities

        idx = int(prc*len(ranges))

        new_ranges = ranges[idx:] + ranges[:idx]
        new_ranges = np.array(new_ranges)
        new_intensities = intensities[idx:] + intensities[:idx]

        new_message = msg
        # new_ranges[new_ranges == np.inf] = np.random.uniform(0, 0.5)
        new_message.ranges = new_ranges
        new_message.intensities = new_intensities

        self.pub.publish(new_message)

s = ScanShifter()
