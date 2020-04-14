#!/usr/bin/env python

import tf
import rospy
import random
import time, os
import numpy as np
from skimage.morphology import binary_dilation, square
from geometry_msgs.msg import PoseStamped, PointStamped, PolygonStamped, Point
from utils import LineTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from math import atan2,sin,cos,sqrt
from scan_simulator_2d import PyScanSimulator2D
from nav_msgs.msg import OccupancyGrid


class RRT():
	"""
	RRT class for running RRT algorithm
	"""
	
	def __init__(self,line_pub,scan_sim):
		self.x_dim = 87
		self.y_dim = 65
		self.count = 0
		self.RADIUS =1
		self.node_limit = 5000
		self.pub = line_pub
		self.scan_sim = scan_sim

        def dist_to_objects(self,node,other):
                theta = atan2(other.point.y-node.point.y, other.point.x - node.point.x)
                dist = self.scan_sim.scan(np.array([[node.point.x,node.point.y,theta]]))[0][0]
                return dist

	def node_collides(self,node,other):
		"""
		Check if a node collides with its environment
		??????
		"""
		#raytracing will go here
                #print("ahhh")
		#theta = atan2(other.point.y-node.point.y,other.point.x-node.point.x)
                #print("ahh2")
		#dist = self.scan_sim.scan(np.array([[node.point.x, node.point.y,theta]]))[0][0]
                #print("ahhh3")
		return self.dist_to_objects(node,other) < node.dist(other)

	def get_rand_node(self):
		p = Point()
		x = random.random()*self.x_dim-60.0
		y = random.random()*self.y_dim-16.9
		#collision here
		#if doesn't collide, return Point
		p.x = x
		p.y = y
		return Node(p,None)

	def line_pub(self,pnts):
		mark = Marker()
		mark.header.frame_id = "map"
		mark.header.stamp = rospy.Time.now()
		mark.type = Marker.LINE_STRIP
		mark.id = self.count
		mark.scale.x = 0.3
		mark.scale.y = 0.5
		mark.scale.z = 0.5
		mark.points = pnts
		mark.color = ColorRGBA(0,0,0,1)
		self.pub.publish(mark)
		time.sleep(0.0)


	def run(self,start,end):
		"""
		runs rrt
		"""
		

		print("Entering rrt run")
		start_node = Node(start,None)
		end_node = Node(end,None)
		state = "Build"
		nodes = [start_node]
		result  = []
		done = False

		while not done:
		    if state == "Finished":
		        current = end_node #???
		        print("Done")
                        result.append(current.point)
		        while current.parent !=None:
		            #draw lines here, final path, note done here
		            current = current.parent
		            result.append(current.point)
		            

		        done = True
		    elif state == "Build":
		        self.count +=1
		       #print("Building tree.....")
		        if self.count < self.node_limit:
		            new_found = False
		            while not new_found:
		                new_n =  self.get_rand_node()
		                parent = nodes[0]
                                best = 100000
                                for n in nodes:
                                #find closest node to new node
		                    if new_n.dist(n) <= best:
                                        #print("test1")
		                        if not self.node_collides(n,new_n): #collision later
		                            parent = n
		                            new_found = True
                                            best = new_n.dist(n)
		                        	#test collision

                        #new_n = parent.step_from_to(new_n,end_node,self.dist_to_objects(parent,new_n))
                        new_n = parent.step_from_to(new_n,end_node)
                        new_n.parent = parent
                        nodes.append(new_n)
                        #if (new_n == parent):
                        #print("SOMETHING IS WRONG RIGHT NOW")
		        #draw line here from parent point to new point
                        #self.line_pub([new_n.point,parent.point])

                        if end_node.collision(new_n,self.RADIUS):
		                state = "Finished"
                                end_node = new_n                                

		    else:
		        print("Exceeded number of nodes")
                return result[::-1]

class Node():
    """
    Node class, will represent pixels in the image
    Each node has:
    -a "point obeject", a tuple representing x and y values      
    -a parent node, which it will connect to
    """
    ID = 0
    def __init__(self, point, parent):
        self.point = point
        self.parent = parent
        self.delta = 4
        self.id = Node.ID
        Node.ID +=1
    def dist(self,other):
        """
        Euclidean distance to another node

        Returns:
            Float representing distance
        """
        #print(self.point)
        #print(self.point.y)
        #print(type(self.point.y))
        #print("OTHER")
        #print(other.point)
        return sqrt((self.point.x - other.point.x)**2+(self.point.y - other.point.y)**2)

    def collision(self,other,collision_dist):
        """
        A node collides with another node if the distance between them
        is less than collision_dist

        Returns:
            Boolean (True if collision, False otherwise)
        """
        return self.dist(other) <= collision_dist

    def step_from_to(self,other,goal, delta = 2):
        """
        Steps from one node to another, creates a new node object if necessary

        Returns:
            A node object??? point??? node for now
        """

        
        #if self.dist(goal) > other.dist(goal):
        #    return other

        if self.dist(other) < self.delta:
            return other
        else:#what does this do?
            """
            d_to_goal = self.dist(goal)
            if self.delta > d_to_goal:
                delta = d_to_goal
            """
            theta = atan2(other.point.y-self.point.y,other.point.x-self.point.x)
            p = Point()
            p.x = self.point.x+delta*cos(theta)
            p.y = self.point.y+delta*sin(theta)
            return Node(p,None)





class BuildTrajectory(object):
    """ Listens for points published by RViz and uses them to build a trajectory. Saves the output to the file system.
    """
    def __init__(self):
        self.save_path = os.path.join(rospy.get_param("~save_path"), time.strftime("%Y-%m-%d-%H-%M-%S") + ".traj") #%Y-%m-%d-%H-%M-%S
        self.trajectory = LineTrajectory("/built_trajectory")
        '''
        Insert appropriate subscribers/publishers here
        
        '''
        self.data_points = []
        self.count = 0
        self.num_waypoints = 4
        self.waypoints = []
        self.click_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_pose, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PolygonStamped, queue_size=10)
        self.trajectory_points = rospy.Publisher("/traj_pts", Marker, queue_size=20)
        self.trajectory.publish_viz() #duration=40.0
        self.rtt_pub = rospy.Publisher("/rtt/final", Marker, queue_size = 10)
        self.rtt_tree_pub = rospy.Publisher("/rtt/tree", Marker, queue_size = 10000)


        # save the built trajectory on shutdown
        rospy.on_shutdown(self.saveTrajectory)

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
                                            2, # number of beams to send out
                                            0, # field of view centered around theta = 0
                                            0, # don't add noise
                                            0.01, # used as an epsilon      
                                            500) # discretize the theta space for faster ray tracing

        #subscribe to map
        self.map_set = False
        self.permissible_region = None
        self.resolution = None
        self.origin = None
        self.width = None
        self.height = None
        self.permissible_indices = None
        rospy.Subscriber(
            "/map",
            OccupancyGrid,
            self.map_callback,
            queue_size = 1)

        self.rrt = RRT(self.rtt_tree_pub,self.scan_sim)

    def map_callback(self,map_msg):
		"""
        #convert the map to a numpy array
		print('HI')
		self.resolution = map_msg.info.resolution
		self.width = map_msg.info.width
		self.height = map_msg.info.height
		#ccmap_ = np.array(map_msg.data, np.double)/100
		#map_ = np.clip(map_,0,1)
        
        #print(map_.shape)
		"""
		
		# assign map-based attributes
		self.resolution = map_msg.info.resolution
		self.width = map_msg.info.width
		self.height = map_msg.info.height

		# Convert the origin to a tuple
		origin_p = map_msg.info.origin.position
		origin_o = map_msg.info.origin.orientation
		origin_o = tf.transformations.euler_from_quaternion((
		        origin_o.x,
		        origin_o.y,
		        origin_o.z,
		        origin_o.w))
		self.origin = (origin_p.x, origin_p.y, origin_o[2])

		# 0: permissible, -1: unmapped, 100: blocked
		array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))

		# 0: not permissible, 1: permissible
		self.permissible_region = np.zeros_like(array_255, dtype = bool)
		self.permissible_region[array_255 == 0] = 1

		# dilate the maps
		self.permissible_region = 1 - binary_dilation(1 - self.permissible_region, square(15))
		scan_map = 1 - np.array(self.permissible_region.reshape((self.permissible_region.size,)), np.double)

		# Initialize a map with the laser scan
		self.scan_sim.set_map(
		        scan_map,
		        map_msg.info.height,
		        map_msg.info.width,
		        map_msg.info.resolution,
		        self.origin,
		        0.5) # Consider anything < 0.5 to be free

		self.permissible_indices = [(r, c) for r in xrange(self.height) for c in xrange(self.width) if self.permissible_region[r][c] == 1]

		self.map_set = True
		print("Map initialized")
            

    def publish_trajectory(self):
        self.traj_pub.publish(self.trajectory.toPolygon())

    def saveTrajectory(self):
        self.trajectory.save(self.save_path)

    def clicked_pose(self,msg):
        """
        self.trajectory.addPoint(point)
        self.data_points.append(point)
        self.mark_pt(self.trajectory_points, (0,1,0), self.data_points)
        if self.count > 2:
            rospy.loginfo("PUBLISH TRAJ")
            print("publish traj")
            self.publish_trajectory()
        """

        point = Point()
        point.x = msg.point.x
        point.y = msg.point.y
        self.waypoints.append(point)
        self.mark_pt(self.trajectory_points, (0,0,1), [point])
        self.count +=1
        print("Point detected")
        print(self.count)
        print(self.waypoints)
        if self.count == self.num_waypoints:
            #do stuff
            self.data_points = []
            start = self.waypoints[0]
            for pt in self.waypoints[1:]:
                rospy.loginfo("Building rrt")
                result = self.rrt.run(start,pt)
                self.data_points = self.data_points + result
                start = result[-1]

            print("Publishing")
            print(result)
            mark = Marker()
            mark.header.frame_id = "/map"
            mark.header.stamp = rospy.Time.now()
            mark.type = Marker.LINE_STRIP
            mark.ns = "final_rtt_path"
            mark.id = 9000
            mark.scale.x = 0.5
            mark.scale.y = 0.5
            mark.scale.z = 0.5
            mark.points = self.data_points
            mark.color = ColorRGBA(1,0,0,1)
            self.rtt_pub.publish(mark)
            print("published...")
            self.count = 0
            self.waypoints = []


    def mark_pt(self, subscriber, color_tup, data):
        mark_pt = Marker()
        mark_pt.header.frame_id = "/map"
        mark_pt.id = self.count
        mark_pt.header.stamp = rospy.Time.now()
        mark_pt.type  = mark_pt.SPHERE_LIST
        mark_pt.action = mark_pt.ADD
        mark_pt.scale.x = 1.0
        mark_pt.scale.y = 1.0
        mark_pt.scale.z= 1.0
        mark_pt.color.a =1.0
        mark_pt.color.r=color_tup[0]
        mark_pt.color.g = color_tup[1]
        mark_pt.color.b = color_tup[2]
        mark_pt.points = data
        subscriber.publish(mark_pt)


if __name__=="__main__":
    rospy.init_node("build_trajectory")
    pf = BuildTrajectory()
    rospy.spin()
