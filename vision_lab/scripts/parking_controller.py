#!/usr/bin/env python
import rospy
from lab4.msg import cone_location, parking_error
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import math

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_cone", cone_location, 
            self.relative_cone_callback)    
        self.drive_pub = rospy.Publisher("/drive", 
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            parking_error, queue_size=10)

        self.relative_x = 0
        self.relative_y = 0

        # Controller params
        self.K_speed = 0.5
        self.K_angle = 0.5

        self.parking_distance = 0.75 # meters


    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()
        
        #################################
        # Play with this number too
        #parking_distance = .75 #meters
        
        # Your Code Here.
        # Use relative position and your control law to populate
        # drive_cmd. 

        est_dis = math.sqrt(self.relative_x**2 + self.relative_y**2) # estimated distance from cone
        est_ang = math.atan2(self.relative_y, self.relative_x) # estimated driving angle from cone
        print('est_dis:', est_dis)
        print('est_ang:', est_ang)
        
        dis_t = 0.1 # distance threshold
        ang_t = 0.05 # angle threshold
        if abs(est_dis - self.parking_distance) < dis_t and abs(est_ang) > ang_t: # if robot is almost the correct distance away but not facing the cone
            speed = -1 * self.K_speed * (5-abs(est_dis - self.parking_distance))
            steering_angle = self.K_angle * est_ang * -1
        else:
            speed = self.K_speed * (est_dis - self.parking_distance) # adjusting speed
            steering_angle = self.K_angle * est_ang # adjusting steering angle:w
            if est_dis < self.parking_distance: steering_angle *= -1 # turn the other direction if robot too close to cone and has to drive backwards
        
        #drive_cmd.drive.speed = speed
        if speed > 0:
            drive_cmd.drive.speed = min(speed, 1) # capping max velocity to be more like real robot
        else:
            drive_cmd.drive.speed = max(speed, -1)
        drive_cmd.drive.steering_angle = steering_angle
        
        #################################

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()
        
    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = parking_error()
        
        #################################
        
        # Your Code Here
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)i
        
        est_ang = math.atan2(self.relative_y, self.relative_x)
        est_dis = math.sqrt(self.relative_x**2 + self.relative_y**2)

        error_msg.x_error = est_dis * math.cos(est_ang) - self.parking_distance # x coord when car is parked in front of cone is parking_distance away
        error_msg.y_error = est_dis * math.sin(est_ang)
        error_msg.distance_error = est_dis - self.parking_distance

        #################################

        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
