#!/usr/bin/env python2

import rospy
from math import pi, cos, sin
import numpy as np
from tf import transformations

class MotionModel:

    def __init__(self):

        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.
        self.prev_time = 0

        rospy.loginfo('motion model initialized')

        ####################################

    def evaluate(self, particles, odometry, noise=0.5):
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            *** PASS IN AS NUMPY ARRAY ***

            odometry: A 3-vector [dx dy dtheta]

            noise:  standard deviation of noise
                    if don't want noise, pass in noise=False

        returns:
            particles: An updated matrix of the
                same size
        """

        ####################################
        dx, dy, dtheta = odometry
        mat_size = particles.shape # size of matrix of particles

        # array of theta values for all of the particles
        theta = particles[:, 2]
        # update particle x, y, theta based on
        # [dx, dy, dtheta] in previous particle frame is multiplied by matrix
        # [cos(theta), -sin(theta), 0]
        # [sin(theta),  cos(theta), 0]
        # [0,           0         , 1]
        # then add dx, dy, dtheta in new frame to x, y, theta of each particle
        updated_x = particles[:, 0] + dx*np.cos(theta) - dy*np.sin(theta)
        updated_y = particles[:, 1] + dx*np.sin(theta) + dy*np.cos(theta)
        updated_theta = theta + dtheta

        # horizontal stack to concatenate each element in updated particle coordinates to form new particles
        # if pass in a standard deviation value for noise, add noise to each particle coordinate
        if not noise:
            return np.stack((updated_x, updated_y, updated_theta), axis=1)
        else:
            return np.stack((updated_x, updated_y, updated_theta), axis=1) + np.random.normal(0, noise, mat_size)

        ####################################

    def get_delta_x(self, odom_data):
        '''
        calculates delta x from Odometry data
        :param odom_data: odometry data (type - Odometry)
        :return: [dx, dy, dtheta] in robot frame
        '''
        '''
        Using pose to find delta x
        pose is the twist value (instantaneous velocity) integrated over time for as long as the odometry topic has been active; this value is equal to ground truth pose in simulator but not on real car
        '''
        # dx = odom_data.pose.pose.position.x
        # dy = odom_data.pose.pose.position.y
        # q = odom_data.pose.pose.orientation # quaternion
        # dtheta = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        # delta_x = [dx, dy, dtheta]

        # Using twist to find delta x
        frame_time = odom_data.header.stamp.secs + odom_data.header.stamp.nsecs*10**(-9) # total time in seconds from time stamp of odometry data

        # if it is the first iteration, don't update the particles so the next iteration will have an accurate prev_time
        if self.prev_time == 0:
            self.prev_time = frame_time
            return [0, 0, 0]

        x_vel = odom_data.twist.twist.linear.x # x linear velocity
        y_vel = odom_data.twist.twist.linear.y # y linear velocity
        ang_vel = odom_data.twist.twist.angular.z # angular velocity

        # calculating delta x in robot frame
        dx = x_vel * (frame_time - self.prev_time)
        dy = y_vel * (frame_time - self.prev_time)
        dtheta = ang_vel * (frame_time - self.prev_time)

        # save for next iteration
        self.prev_time = frame_time

        # return delta_x from odom data in robot frame
        return [dx, dy, dtheta]

if __name__ == '__main__':
    # for local testing
    m = MotionModel()
