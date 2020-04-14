#!/usr/bin/env python2

import numpy as np
import math

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

# for drawing wall being followed
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    def __init__(self):
        # Initialize your publishers and
        # subscribers here
        self.sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)

        self.marker_pub = rospy.Publisher('visualization_maker', Marker, queue_size=10) # marker publisher

        # for PD controller

        self.Kp = 0.5 # TODO: determine good K values
        self.Kd = 1


    def callback(self, data):
        '''
        publish autonomous drive data to DRIVE_TOPIC
        :param data: LaserScan data from SCAN_TOPIC
        '''
        # *** making the assumption that angle_max > 0 and angle_min < 0 ***
        # want to sense ahead of the robot so it can adjust for upcoming wall
        # slice range from r_min to r_max or -r_max to -r_min

        r_min = 0
        r_max = math.pi/3

        select_range, ang_min = self.slice_range(data, r_min, r_max, self.SIDE)

        #****************** drawing sensed wall regression  ***************
        #x = []
        #y = []
        #for i in range(len(select_range)):
        #    x.append(self.polar_to_cart(select_range[i], ang_min+data.angle_increment*i, coor='x'))
        #    y.append(self.polar_to_cart(select_range[i], ang_min+data.angle_increment*i, coor='y'))

        x_y = [] # list of [x, y] coords
        for i in range(len(select_range)):
            if select_range[i] > 80: # filter out inf
                continue
            x = self.polar_to_cart(select_range[i], ang_min+data.angle_increment*i, coor='x')
            y = self.polar_to_cart(select_range[i], ang_min+data.angle_increment*i, coor='y')
            #x_y.append(self.rotate((x, y), 0))
            x_y.append([x, y])

        points, est_dis, slope = self.linear_reg(x_y)
        #self.draw_wall(points, c=(1.0,1.0,1.0,1.0)) # plot linear regression with respect to the robot's coordinate frame

        rotated_points = [self.rotate(p, math.pi/3) for p in x_y]
        rotated_line_points, r_est_dis, r_slope = self.linear_reg(rotated_points)
        self.draw_wall(rotated_line_points, c=(1.0,1.0,1.0,1.0)) # linear regression in the velodyne's coordinate frame
        #self.draw_wall([Point(coord[0], coord[1], 0) for coord in x_y], c=(1.0,1.0,1.0,1.0)) # plot sensed and rotated points

        #********************* PD controller *******************
        error = self.DESIRED_DISTANCE - est_dis

        if self.SIDE > 0: # +1 represents the left wall
            derivative = -slope/(1+abs(slope))
            output = -(error*self.Kp + derivative*self.Kd)
        else: # -1 represents the right wall
            derivative = slope/(1+abs(slope))
            output = error*self.Kp + derivative*self.Kd

        rospy.loginfo(est_dis)
        #rospy.loginfo(output)

        # **************** AckermannDriveStamped ******************
        drive_data = AckermannDriveStamped()
        drive_data.drive.steering_angle = output
        #drive_data.drive.steering_angle_velocity = 0
        drive_data.drive.speed = self.VELOCITY
        drive_data.drive.acceleration = 0
        drive_data.drive.jerk = 0

        self.pub.publish(drive_data) # publish AckermannDriveStamped data to DRIVE_TOPIC


    def slice_range(self, data, r_min, r_max, side):
        '''
        slices data based on min and max angle ** HAS SHIFT OF PI/3 **
        :param data: laser scan data that contains range data
        :param r_min: min angle to keep in slice
        :param r_max: max angle to keep  in slice
        :param side: +1 for left wall, -1 for right wall
        :return: sliced range data, angle of index 0
        '''
        ranges = data.ranges

        if side > 0: # +1 represents the left wall
            r_min += math.pi/3 # if r_min = 0, then it should now be pi/3
            r_max += math.pi/3 # if r_max = pi/3,  then it should now be 2pi/3
        else: # -1 represents the right wall
            temp = r_min
            r_min = -r_max + math.pi/3 # if r_max was pi/3, then r_min is now 0
            r_max = -temp + math.pi/3 # if r_min was 0, then r_max is now pi/3

        select_range = ranges[self.angle_to_index(data, r_min):self.angle_to_index(data, r_max)]
        return select_range, r_min


    def angle_to_index(self, data, angle):
        '''
        convert angle to index in range list of scan data
        :param data: LaserScan data
        :param angle: angle in radians
        :return: index in range list corresponding to angle
        '''
        return int((angle - data.angle_min)/data.angle_increment)

    def polar_to_cart(self, r, theta, coor="Point"):
        '''
        convert range, angle points to x, y, z points with 0, 0, 0 as baselink origin
        :param r: range (sensed distance)
        :param theta: angle
        :param coor: 'x' for x coordinate
                     'y' for y coordinate
                     'Point' default to Point class (x, y, 0)
        :return: coordinate or Point
        '''
        theta -= math.pi/3
        if coor == 'x':
            return r*math.cos(theta)
        elif coor == 'y':
            return r*math.sin(theta)
        else:
            return Point(r*math.cos(theta), r*math.sin(theta),  0)

    def rotate(self, point, rot_ang):
        '''
        rotate (x, y) point CCW a certain angle about the origin
        :param point: [x, y] list
        :param rot_ang: angle to rotate by
        :return: rotated [x, y] list
        '''
        rot_matrix = [[math.cos(rot_ang), -math.sin(rot_ang)], [math.sin(rot_ang), math.cos(rot_ang)]]
        return np.matmul(rot_matrix, point)

    def crop_middle(self, points, buf=20):
        '''
        get rid of points in the middle of the list
        :param points: list of [x, y] coords
        :param buf: how many points to save on each end of the list
        '''
        return points[:buf] + points[-buf:]

    def linear_reg(self, x_y):
        '''
        calculates linear regression on x and y values passed in as lists
        :param x_y: list of [x, y] coords
        :return: list of Points for linear regression, distance from origin (baselink) to line, angle between x-axis and line (robot angle from wall)
        '''
        x, y = list(zip(*x_y))

        coeffs = np.polynomial.polynomial.polyfit(x, y, 1, w=[1/(x[i]**2+y[i]**2) for i in range(len(x))]) # weight really far points to have less of a contribution to the line
        #coeffs = np.polynomial.polynomial.polyfit(x, y, 1)

        slope = coeffs[1]
        intercept = coeffs[0]

        points = []
        for x_coor in x:
            points.append(Point(x_coor, slope*x_coor+intercept, 0))

        dis = abs(intercept)/math.sqrt(slope**2+1)

        return points, dis, slope


    def draw_wall(self, points, c=(1.0, 0.0, 0.0, 1.0)):
        '''
        draws marker line in rviz based on list of points
        :param points: list of points
        :param c: RGBA color tuple; defaults to red
        '''
        marker = Marker()
        marker.id = 0
        marker.header.frame_id = 'base_link'
        marker.type = Marker.LINE_STRIP
        marker.lifetime = rospy.Duration(60)
        marker.color = ColorRGBA(*c)

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0

        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        marker.points = points

        self.marker_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
