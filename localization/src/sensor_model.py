#!/usr/bin/env python2

import numpy as np
from scan_simulator_2d import PyScanSimulator2D
import pickle
import math

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler

class SensorModel:


    def __init__(self):

        # Fetch parameters
        self.map_topic = rospy.get_param("~map_topic")
        self.num_beams_per_particle = rospy.get_param("~num_beams_per_particle")
        self.scan_theta_discretization = rospy.get_param("~scan_theta_discretization")
        self.scan_field_of_view = rospy.get_param("~scan_field_of_view")

        ####################################
        # TODO
        # Precompute the sensor model here
        # (You should probably write a
        #  function for this)

        #with open('index_data.pkl', 'rb') as f: # *** MAKE SURE PKL FILE IS IN src DIRECTORY ***
        with open('/home/racecar/racecar_ws/src/localization/sensor_model/index_data.pkl', 'rb') as f:
            self.table = pickle.load(f) # lookup table for probabilities
        # key = (zt, zt_star) = (measured distance, ground truth distance)
        # value = probability(measured | ground truth)

        def prob_lookup(zt, zt_star):
            '''
            look up probability given zt (measured) and zt_star (ground truth)
            floor indices to match values stored in the table
            -- zt
                min: 0.5
                max: 100
                num_samples: 200
            -- zt_star
                min: 0.5
                max: 100
                num_samples: 200
            :param zt: measured distance (m)
            :param zt_star: ground truth distance
            :return: probability (zt | zt_star)
            '''
            n = 200
            zmax = 100
            # TODO: use round() instead of math.floor()? Will it have an index out of range error if rounds up at max index?
            zt_index = int(math.floor((zt - 0.5)*(n-1)/(zmax-0)))
            zt_star_index = int(math.floor((zt_star-0.5)*(n-1)/(zmax-0.1)))
            return self.table[zt_index][zt_star_index]

        # self.prob_lookup is a function that can take a nested sequence of objects or numpy arrays as inputs and return a single numpy array as output
        self.prob_lookup = np.vectorize(prob_lookup)

        ####################################

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
                self.num_beams_per_particle,
                self.scan_field_of_view,
                0, # This is not the simulator, don't add noise
                0.01, # This is used as an epsilon
                self.scan_theta_discretization)

        # Subscribe to the map
        self.map_set = False
        rospy.Subscriber(
                self.map_topic,
                OccupancyGrid,
                self.map_callback,
                queue_size=1)

        rospy.loginfo('sensor model initialized')

    def evaluate(self, particles, observation, downsample=20, sim=True):
        """
        Evaluate how likely each particle is given
        the observed scan.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            observation:    LaserScan message
                            for a vector of lidar data of length m, use observation.ranges

            downsample: how many indicies to skip by during the slice to downsample rays from each particle

            sim: boolean indicating if working in sim; defaults to True

        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
               type - numpy array
        """

        if not self.map_set:
            rospy.loginfo('map not set in sensor model (evaluate function)')
            return

        ####################################
        # TODO
        # Evaluate the sensor model here!
        #
        # You will probably want to use this function
        # to perform ray tracing from all the particles.
        # This produces a matrix of size N x num_beams_per_particle
        if sim == False:
            observation = self.slice_ranges(observation)
        elif sim == True:
            observation = observation.ranges

        scans = self.scan_sim.scan(particles)

        scans = scans[:,::downsample]
        observation = observation[::downsample]

        probabilities = self.prob_lookup(observation, scans)

        # probabilities is a 2D array where each subarray contains probability of each beam
        # take product of each subarray to find probability of particle
        return np.prod(probabilities, axis=1)
        ####################################

    def slice_ranges(self, scan_data):
        '''
        slice the correct field of view on the velodyne lidar ranges
        then downsample range data from car lidar to self.num_beams_per_particle
        :param scan_data: LaserScan data from car
        :return: range data as list of length self.num_beams_per_particle
        '''
        # want slices from -75 to 180 degrees appended to -180 to -165 degrees
        ranges = scan_data.ranges[self.ati(-75*math.pi/180):] + scan_data.ranges[:self.ati(-165*math.pi/180)]

        # downsample ranges to length of self.num_beams_per_particle
        indices = np.round(np.linspace(0, len(ranges)-1, self.num_beams_per_particle)).astype(int)
        return np.array(ranges)[indices]

    def ati(self, data, angle):
        '''
        convert angle to index in range list of scan data
        :param data: LaserScan data
        :param angle: angle in radians
        :return: index in range list corresponding to angle
        '''
        return int((angle - data.angle_min)/data.angle_increment)

    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        map_ = np.array(map_msg.data, np.double)/100.
        map_ = np.clip(map_, 0, 1)

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = tf.transformations.euler_from_quaternion((
                origin_o.x,
                origin_o.y,
                origin_o.z,
                origin_o.w))
        origin = (origin_p.x, origin_p.y, origin_o[2])

        # Initialize a map with the laser scan
        self.scan_sim.set_map(
                map_,
                map_msg.info.height,
                map_msg.info.width,
                map_msg.info.resolution,
                origin,
                0.5) # Consider anything < 0.5 to be free

        # Make the map set
        self.map_set = True

        print("Map initialized")

if __name__ == '__main__':
    s = SensorModel()
