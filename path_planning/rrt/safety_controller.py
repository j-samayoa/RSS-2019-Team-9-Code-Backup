#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
from math import pi

class SafetyController:
    def __init__(self):
        self.scan_sub  = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.pure_pursuit_sub = rospy.Subscriber('/segment_distance',Float32,self.test_pursuit)
        #self.drive_sub = rospy.Subscriber('/vesc/high_level/ackermann_cmd_mux/output',AckermannDriveStamped, callback)
        self.safety_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/safety',AckermannDriveStamped,queue_size = 20)
        self.dist_pub = rospy.Publisher('/min_dist',Float32,queue_size = 20)
        self.min_dist = 0.5
        self.VELOCITY = rospy.get_param("/speed") #may change to odom later
        self.SIDE = 1
		self.panic = False
		self.safety_pub = rospy.Publisher('/panic_flag',bool,queue_size = 20)

	def test_pursuit(self,data):
		self.panic = data > 3		
		
    def calc_distance(self,car_angle,data):
         i = self.angle_to_index(data,car_angle+pi/3)
         temp = data.ranges[i-20:i+21]
         num = 0
         for r in temp:
             if r < self.min_dist or r == float("Inf"):
                 num+=1
         if num > 10:
            return True


    def angle_to_index(self, data, angle):
        '''
        convert angle to index in range list of scan data
        :param data: LaserScan data
        :param angle: angle in radians
        :return: index in range list corresponding to angle
        '''
        return int((angle - data.angle_min)/data.angle_increment)
	
	def particle_filter_running():
		topics = rospy.get_published_topics()
		for topic, type in topics:
			if topic == "/pf/viz/inferred_pose":
				return True
		return False


    def callback(self,data):
        def stop(angle,vel = self.VELOCITY/2):
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "base_link"
            drive_msg.drive.speed = vel
            drive_msg.drive.steering_angle = angle
            self.safety_pub.publish(drive_msg)


        self.dist_pub.publish(min(data.ranges))
        #rospy.loginfo("min dist: %s"%min(data.ranges))

        front = self.calc_distance(0,data)
        left_side = self.calc_distance(pi/2,data)
        right_side = self.calc_distance(-pi/2,data)
        #if front or right_side or left_side:
        #    stop()

		if not particle_filter_running():
			stop(0.0,0.0)
		elif self.panic:
			stop(0.0,0.0)
        elif front:
            print("FRONT")
            angle =self.SIDE*pi/2
            stop(angle,vel = -self.VELOCITY/2)
        elif left_side:
            print("LEFT")
            stop(-pi/2)
        elif right_side:
            print("RIGHT")
            stop(pi/2)



if __name__ == '__main__':
    rospy.init_node("safety_controller")
    safetyController = SafetyController()
    rospy.spin()


