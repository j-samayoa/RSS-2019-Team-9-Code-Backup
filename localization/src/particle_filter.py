#!/usr/bin/env python2

import rospy
from tf import transformations, TransformBroadcaster
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point, PoseArray, Pose, Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Float32

from sensor_model import SensorModel
from motion_model import MotionModel

import numpy as np
from threading import Lock

from time import time

'''
example use of a thread:
from threading import Thread

t = Thread(name='Thread1', target=func_name, args=(arg1, arg2, ...))
t.start()

need threading to prevent motion model and sensor model from updating particles at the same time

or use locks
from threading import Lock

lock = Lock()
lock.acquire()
lock.release()
'''

class ParticleFilter:

    # toggle robot driving in a circle in the simulator
    DRIVING = True
    # set to True if working in the simulator, else False
    SIM = True

    """
    *** IF WORKING ON THE REAL ROBOT: ***
    - in params.yaml:
        - change particle_filter_frame from 'base_link_pf' to 'base_link'
        - change odom_topic from '/odom' to '/vesc/odom'
    - in localize.launch:
        - uncomment out region that initializes map server to use stata basement map
    - use SIM = False and DRIVING = False
    """

    def __init__(self):
        # using locks so threads can't access self.particles at the same time (avoiding race condition)
        self.lock = Lock()

        self.lock.acquire()

        # Get parameters
        self.particle_filter_frame = \
                rospy.get_param("~particle_filter_frame") # should be '/base_link_pf' in sim and '/base_link' on car

        # Initialize the models
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()

        # Topics
        self.odom_topic = rospy.get_param("~odom_topic")
        self.scan_topic = rospy.get_param("~scan_topic")
        self.particle_marker_topic = '/particle_marker'
        self.click_topic = '/move_base_simple/goal'
        self.pose_array_topic = '/pose_array'
        self.inferred_pose_topic = '/inferred_pose'

        self.error_x_topic = '/error_x'
        self.error_y_topic = '/error_y'
        self.error_t_topic = '/error_t'

        # Subscribers and Publishers
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback) # '/odom' for sim and 'vesc/odom' for car
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)

        self.tf_broadcaster = TransformBroadcaster()

        # Listening to clicks in rviz and publishing particle markers to rviz
        rospy.Subscriber(self.click_topic, PoseStamped, self.click_callback)
        self.particle_marker_pub = rospy.Publisher(self.particle_marker_topic, Marker, queue_size=1)
        self.pose_array_pub = rospy.Publisher(self.pose_array_topic, PoseArray, queue_size=1)
        self.inferred_pose_pub = rospy.Publisher(self.inferred_pose_topic, Marker, queue_size=1)

        self.error_x_pub = rospy.Publisher(self.error_x_topic, Float32, queue_size=1)
        self.error_y_pub = rospy.Publisher(self.error_y_topic, Float32, queue_size=1)
        self.error_t_pub = rospy.Publisher(self.error_t_topic, Float32, queue_size=1)

        self.rate = rospy.Rate(30) # should be greater than 20 Hz

        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz
        #
        # Publish a transformation frame between the map
        # and the particle_filter_frame.

        # Initializing particles and weights
        self.num_particles = rospy.get_param('~num_particles')
        # currently intializing all particles as origin
        self.particles = np.zeros((self.num_particles, 3))
        # all particles are initially equally likely
        self.weights = np.full((self.num_particles), 1./self.num_particles)

        # for error calculations
        self.groundtruth = [0, 0, 0]

        # send drive commands to test in sim
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.drive.steering_angle = 0
        self.drive_msg.drive.speed = 1

        if self.SIM == False: # don't toggle on driving if working on real robot
            self.DRIVE = False

        self.publish_transform()

        self.lock.release()

    def odom_callback(self, odom_data):
        '''
        whenever you get odometry data, use the motion model to update the particle positions
        determine the "average" particle pose and publish that transform
        :param odom_data: odometry data (type - Odometry)
        :return: no return, but updates self.particles positions and publishes transform
        '''
        self.lock.acquire()
        #start = time()

        x = odom_data.pose.pose.position.x
        y = odom_data.pose.pose.position.y
        q_w = odom_data.pose.pose.orientation.w
        q_z = odom_data.pose.pose.orientation.z
        euler = transformations.euler_from_quaternion(np.array([0, 0, q_z, q_w]))
        self.groundtruth = [x, y, euler]

        self.particles = self.motion_model.evaluate(self.particles, self.motion_model.get_delta_x(odom_data), noise=0.2)

        self.publish_transform()


        #end = time()
        #print('odom_callback time:', end-start)
        self.lock.release()

    def scan_callback(self, scan_data):
        '''
        whenever you get sensor data, use the sensor model to compute the particle probabilities. Then resample the particles based on these probabilities
        determine the "average" particle pose and publish that transform
        :param scan_data: scan data (type - LaserScan)
        :return: no return, but updates self.particles by resampling particles and publishes transform
        '''
        self.lock.acquire()

        probabilities = self.sensor_model.evaluate(self.particles, scan_data, downsample=20, sim=self.SIM)

        # if map wasn't initialized in sensor_model, probabilites=None
        if isinstance(probabilities, type(None)):
            self.lock.release()
            rospy.loginfo('no probabilities to use in scan_callback')
            return

        smooth_p = np.power(probabilities, [1/3.]*len(probabilities))

        self.weights = smooth_p/np.sum(smooth_p) # normalize weights so they sum to 1

        # resample particles based on weights
        self.particles = self.resample_particles()

        self.publish_transform()

        self.lock.release()

    def resample_particles(self):
        '''
        reseample particles based on their probability weights
        allow repeats of particles; to not allow, use replace=False
        '''
        # choose with indices in particles array to keep using probabilities as weights
        indices = np.random.choice(self.num_particles, self.num_particles, p=self.weights)
        # return particles with chosen indices
        return self.particles[indices]

    def publish_transform(self):
        '''
        anytime the particles are updated, determine the 'average' particle pose and broadcast that transform
        :return: no return, but broadcast transform between 'map' frame and self.particle_filter_frame
        '''
        x_avg, y_avg, theta_avg = self.avg_pose()

        q = transformations.quaternion_from_euler(0, 0, theta_avg)

        # broadcast transformation
        self.tf_broadcaster.sendTransform((x_avg, y_avg, 0), q, rospy.Time.now(), self.particle_filter_frame, 'map')

        self.publish_pose_array()
        #self.publish_particle_markers()

        # Driving in the simulator
        if self.DRIVING:
            self.drive_pub.publish(self.drive_msg)

        # publish inferred position in rviz as arrow
        if self.inferred_pose_pub.get_num_connections > 0:
            a = Marker()
            a.header.frame_id = 'map'
            a.type = Marker.ARROW
            a.color = ColorRGBA(0, 1, 1, 1)

            a.scale.x = 0.5
            a.scale.y = 0.1
            a.scale.z = 0.1

            q = transformations.quaternion_from_euler(0, 0, theta_avg)
            quat = Quaternion()
            quat.x = q[0]
            quat.y = q[1]
            quat.z = q[2]
            quat.w = q[3]

            a.pose.position.x = x_avg
            a.pose.position.y = y_avg
            a.pose.orientation = quat

            self.inferred_pose_pub.publish(a)

        # for error plots
        try:
            x, y, theta = self.groundtruth
            x_error = x_avg - x
            y_error = y_avg - y
            t_error = theta_avg - theta[2]
            self.error_x_pub.publish(x_error)
            self.error_y_pub.publish(y_error)
            self.error_t_pub.publish(t_error)
        except Exception as e:
            rospy.loginfo(e)

    def avg_pose(self):
        '''
        determine the average particle pose
        ** have to use mean of circular quantities for theta
        ** distirbution might be multi modal
        :param: no params, but uses instance variable self.particles
        :return: average particle pose as [x, y, theta]
        '''
        x = np.mean(self.particles[:, 0])
        y = np.mean(self.particles[:, 1])
        # circular mean is angle between average x component and y component of the angles data
        theta = np.arctan2(np.mean(np.sin(self.particles[:, 2])), np.mean(np.cos(self.particles[:, 2])))

        return [x, y, theta]

    def click_callback(self, pose_data):
        '''
        use click in rviz to initialize particles
        :param pose_data: pose of rviz click (type - PoseStamped)
        use 2D Nav Goal to guess car location
        :return: no return, but updates self.particles
        '''
        self.lock.acquire()

        x = pose_data.pose.position.x
        y = pose_data.pose.position.y
        q = pose_data.pose.orientation
        theta = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        # random points from normal distribution centered at [x, y, theta]
        self.particles = np.random.normal([x, y, theta], 1, (self.num_particles, 3))
        self.weights = np.full((self.num_particles), 1./self.num_particles)

        self.publish_pose_array()
        #self.publish_particle_markers()

        self.lock.release()

    def publish_particle_markers(self):
        '''
        publish particles as markers in rviz
        '''
        # if something is listening for marker publications
        if self.particle_marker_pub.get_num_connections > 0:
            m = Marker()
            m.header.frame_id = 'map'
            m.type = Marker.POINTS
            #m.color = ColorRGBA(1, 0, 0, 1)

            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.2

            for i in range(len(self.particles)):
                p = self.particles[i]
                w = self.weights[i]
                m.points.append(Point(p[0], p[1], 0))
                m.colors.append(self.prob_to_color(w))

            self.particle_marker_pub.publish(m)

    def prob_to_color(self, prob):
        '''
        convert a probability to a color
        red is high probability
        orange
        yellow
        green
        cyan
        light blue
        blue
        purple
        magenta
        black is low
        :param prob: probability between 0 and 1
        :return: ColorRBGA corresponding to prob
        '''
        m = max(self.weights) # max weight
        if prob > m*0.9:
            return ColorRGBA(1, 0, 0, 1)
        if prob > m*0.8:
            return ColorRGBA(1, 0.5, 0, 1)
        if prob > m*0.7:
            return ColorRGBA(1, 1, 0, 1)
        if prob > m*0.6:
            return ColorRGBA(0, 1, 0, 1)
        if prob > m*0.5:
            return ColorRGBA(0, 1, 1, 1)
        if prob > m*0.4:
            return ColorRGBA(0, 0.5, 1, 1)
        if prob > m*0.3:
            return ColorRGBA(0, 0, 1, 1)
        if prob > m*0.2:
            return ColorRGBA(0.5, 0, 1, 1)
        if prob > m*0.1:
            return ColorRGBA(1, 0, 1, 1)
        else:
            return ColorRGBA(0, 0, 0, 1)

    def publish_pose_array(self):
        '''
        publish arrows in rviz for to visualize particles
        '''
        particles = np.copy(self.particles)
        if self.pose_array_topic > 0:
            pa = PoseArray()
            pa.header.frame_id = 'map'
            for p in particles[:20]:
                pose = Pose()
                pose.position = Point(p[0], p[1], 0)
                q = transformations.quaternion_from_euler(0, 0, p[2])
                quat = Quaternion()
                quat.x = q[0]
                quat.y = q[1]
                quat.z = q[2]
                quat.w = q[3]
                pose.orientation = quat
                pa.poses.append(pose)
            self.pose_array_pub.publish(pa)


if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.loginfo('particle filter running')

    # Test the sensor model
    # print('should be high probability:')
    # print(pf.sensor_model.prob_lookup(15, 15.4))
    # print('should be low probability:')
    # print(pf.sensor_model.prob_lookup(2, 30))

    rospy.spin()
