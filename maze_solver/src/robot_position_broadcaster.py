#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
from std_msgs.msg import Header

if __name__ == '__main__':
    rospy.init_node('robot_position_broadcaster')

    listener = tf.TransformListener()

    robot_position_publisher = rospy.Publisher('/robot_position', geometry_msgs.msg.PoseStamped,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            ((x,y,z),(xx,yy,zz,ww)) = listener.lookupTransform('/cartographer_map', '/base_link', rospy.Time(0))

            p = geometry_msgs.msg.PoseStamped()

            p.header.frame_id = "/cartographer_map"
            p.header.stamp = rospy.Time.now()

            p.pose.position.x=x 
            p.pose.position.y=y 
            p.pose.position.z=z 

            p.pose.orientation.x = xx
            p.pose.orientation.y = yy
            p.pose.orientation.z = zz
            p.pose.orientation.w = ww
            robot_position_publisher.publish(p)

            rate.sleep()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

