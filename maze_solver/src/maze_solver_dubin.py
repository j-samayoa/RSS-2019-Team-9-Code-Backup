#!/usr/bin/env python
import random
import math
from Queue import Queue
import numpy as np
import rospy
import tf
import dubins
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import OccupancyGrid
from utils import *
from tf.transformations import euler_from_quaternion

class MazeSolver():
    def __init__(self):
        rospy.init_node("maze_solver")
        self.R_MAZE = 100 #TODO: this is not actually 100
        self.downsample_factor = 4
        self.occupancy_threshold = 50 #if a cell has a value above this, then it is occupied
        self.width = 0
        self.height = 0
        self.MAP = []
        self.data_points = []
        self.resolution = 0.05 * self.downsample_factor #m/cell
        self.latest_map = None
        self.turning_radius_in_meters = 0.5
        self.turning_radius = self.turning_radius_in_meters/self.resolution
        self.current_position = None
        self.map_pub = rospy.Publisher("/downsampled_map",OccupancyGrid,queue_size=10)
        self.position_sub = rospy.Subscriber("/robot_position",PoseStamped,self.update_position)
        self.sub = rospy.Subscriber("/cartographer_map",OccupancyGrid,self.update_map,queue_size=10)
        self.pub = rospy.Publisher("/graph",MarkerArray,queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current",PolygonStamped,queue_size=10)
        self.trajectory_points = rospy.Publisher("/traj_pts",Marker,queue_size=10)
        self.get_out()

    def on_the_map(self, tile, width, height):
        (r,c) = tile
        if(r < 0 or r>=height):
            return False
        if(c < 0 or c>=width):
            return False
        return True 

    def update_position(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.current_position = (x,y)
        self.current_rotation = msg.pose.orientation
        quaternion = (
            self.current_rotation.x,
            self.current_rotation.y,
            self.current_rotation.z,
            self.current_rotation.w)
        theta = euler_from_quaternion(quaternion)[2]
        self.theta = theta

    def update_map(self,msg):
        self.latest_map = msg

    def publish_nodes(self, nodes):
        if nodes is None:
            return
        marker_array = MarkerArray()
        for i,node in enumerate(nodes):
            marker = Marker()
            marker.header.frame_id = "/cartographer_map"
            marker.header.stamp = rospy.Time.now()
            marker.type = marker.CYLINDER
            marker.action = marker.ADD
            marker.ns = "basic_shapes"
            marker.id = i
            marker.pose.position.x = node[0]
            marker.pose.position.y = node[1]
            marker.pose.position.z = 0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            # if node.is_at_the_border():
            #     marker.color.b = 1.0
            # else:
            marker.color.r = 1.0
            marker_array.markers.append(marker)
        self.pub.publish(marker_array)

    def mark_pt(self, publisher, color_tup, data):
        mark_pt = Marker()
        mark_pt.header.frame_id = "/cartographer_map"
        mark_pt.header.stamp = rospy.Time.now()
        mark_pt.type  = mark_pt.SPHERE_LIST
        mark_pt.action = mark_pt.ADD
        mark_pt.scale.x = .5
        mark_pt.scale.y = .5
        mark_pt.scale.z= .5
        mark_pt.color.a =1.0
        mark_pt.color.r =color_tup[0]
        mark_pt.color.g = color_tup[1]
        mark_pt.color.b = color_tup[2]
        mark_pt.points = data
        publisher.publish(mark_pt)

    def publish_map(self):
        g = OccupancyGrid()
        g.header.frame_id = "/cartographer_map"
        g.info.resolution = self.resolution
        g.info.width = self.width
        g.info.height = self.height
        g.info.origin = self.true_origin
        g.info.map_load_time = rospy.Time.now()
        g.data = self.MAP
        self.map_pub.publish(g)

    def publish_trajectory(self):
        self.traj_pub.publish(self.trajectory.toPolygon())

    def get_current_position(self):
        if self.current_position is None:
            print("CURRENT POSITION NOT AVAILABLE")
            _ = rospy.wait_for_message( "/robot_position", PoseStamped)
        return self.current_position

    def get_current_theta(self):
        if self.theta is None:
            print("CURRENT POSITION NOT AVAILABLE")
            _ = rospy.wait_for_message( "/robot_position", PoseStamped)
        return self.theta

    def to_pixel_coords(self,p):
        origin = self.origin
        theta = -self.rotation
        resolution = self.resolution
        height = self.height
        x,y = rotate(origin,p,theta)
        x = x - origin[0]
        y = y - origin[1]
        col = int(x/resolution)
        row = int(ceiling(y/resolution))
        return (row,col)

    def to_meter_coords(self,p):
        (row,col) = p
        y = row*self.resolution
        x = col*self.resolution
        (x,y) = rotate(self.origin,(x,y),self.rotation)
        (ox,oy) = self.origin
        x = ox + x
        y = oy + y
        return (x,y)

    def get_current_tile(self):
        return self.to_pixel_coords(self.get_current_position())

    def fill_in_gaps(self):
        gaps = [[0 for j in range(self.width)] for i in range(self.height)]
        dr = [-1,-1,-1,0,1,1,1,0]
        dc = [-1,0,1,1,1,0,-1,-1]
        for row in range(self.height):
            for col in range(self.width):
                num_occupied = 0
                for i in range(8):
                    rr = row + dr[i]
                    cc = col + dc[i]
                    if(self.on_the_map((rr,cc),self.width,self.height)):
                        idx = rr*self.width + cc
                        if self.MAP[idx] == 100:
                            num_occupied += 1
                if num_occupied >= 2:
                    gaps[row][col] = 100
        for row in range(self.height):
            for col in range(self.width):
                idx = row*self.width + col
                if gaps[row][col] == 100:
                    self.MAP[idx] = 100

    def get_neighboring_tiles(self,tile):
        r,c = tile
        dr = [-1,-1,-1,0,1,1,1,0]
        dc = [-1,0,1,1,1,0,-1,-1]
        tiles = []
        for i in range(8):
            rr = r + dr[i]
            cc = c + dc[i]
            if(self.on_the_map((rr,cc),self.width,self.height)):
                tiles.append((rr,cc))
            # else:
            #     print("%s is not on the map. width: %s, height: %s"%((rr,cc), self.width, self.height))
        
        tiles_with_angles = []
        for tile in tiles:
            r,c = tile
            for theta in frange(0,3.14,3.14/4):
                tiles_with_angles.append(((r,c),theta))
        return tiles_with_angles

    def idx_of(self,p):
        (r,c) = p
        return r*self.width + c    

    def put_target_on_map(self):
        pass
        n = self.width*self.height
        while(True):
            idx = random.randint(0,n-1)
            if self.MAP[idx] == 0:
                self.MAP[idx] = -1
                return

    def downsample(self, data, width, height, resolution, downsample_factor):
        print("DATA: ",np.array(data).shape)
        df = downsample_factor
        w = ceiling(float(width)/downsample_factor)
        h = ceiling(float(height)/downsample_factor)
        d = [[-1 for j in range(w)] for i in range(h)]
        for r in range(h):
            for c in range(w):
                unexplored = 0
                occupancy  = 0
                total      = 0
                for i in range(downsample_factor):
                    for j in range(downsample_factor):
                        rr = r*downsample_factor + i
                        cc = c*downsample_factor + j
                        if (self.on_the_map((rr,cc),width,height)):
                            total+=1
                            idx = rr*width + cc
                            if (data[idx] == -1):
                                unexplored += 1.0
                            else:
                                if data[idx] > self.occupancy_threshold:
                                    occupancy+=1.0
                if total == 0:
                    continue
                unexplored = unexplored/total
                if total != unexplored:
                    occupancy  = occupancy/(total - unexplored)
                if(occupancy > 0.1): 
                    d[r][c] = 100
                elif(unexplored < 0.5):
                    d[r][c] = 0
        return np.array(d).flatten().tolist()

    def save_map(self):
        if self.latest_map is not None:
            self.width       = self.latest_map.info.width
            self.height      = self.latest_map.info.height
            self.true_origin = self.latest_map.info.origin
            self.original_info = self.latest_map.info
            self.origin      = (self.latest_map.info.origin.position.x, self.latest_map.info.origin.position.y)
            self.resolution  = float(self.latest_map.info.resolution)
            self.MAP         = self.downsample(self.latest_map.data, self.width, self.height, self.resolution, self.downsample_factor)
            self.resolution *= self.downsample_factor
            self.turning_radius = self.turning_radius_in_meters/self.resolution
            self.width       = ceiling(float(self.width)/self.downsample_factor)
            self.height      = ceiling(float(self.height)/self.downsample_factor)
            self.fill_in_gaps()
            # self.put_target_on_map()
            quaternion = (
                self.latest_map.info.origin.orientation.x,
                self.latest_map.info.origin.orientation.y,
                self.latest_map.info.origin.orientation.z,
                self.latest_map.info.origin.orientation.w)
            euler = euler_from_quaternion(quaternion)
            self.rotation = euler[2]
            self.publish_map()
        else:
            print("waiting for cartographer")
            _ = rospy.wait_for_message( "/cartographer_map", OccupancyGrid)
            self.save_map()

    def get_exploration_path(self):
        current_tile = self.get_current_tile()
        theta = self.get_current_theta()
        q = Queue()
        q.put((current_tile, theta))
        visited = set()
        destination = None
        parent  = {}
        parent[current_tile] = current_tile
        distance = {}
        print("current tile: %s")
        print(current_tile)
        for r in range(self.height):
            for c in range(self.width):
                distance[(r,c)] = float('inf')

        distance[current_tile] = 0
        found_destination = False

        while(not q.empty() and not found_destination):
            (current_tile, current_theta) = q.get()
            # print("currently at (%s,%s)"%(current_tile[0],current_tile[1]))
            rr,cc = current_tile
            idx = rr*self.width + cc
            if self.MAP[idx] < 0:
                print("found destination!")
                destination = current_tile
                found_destination = True
                break

            if current_tile in visited:
                continue

            neighbors = self.get_neighboring_tiles(current_tile)
            print("NEIGHBORS: ",neighbors)
            for (neighbor_tile,theta) in neighbors:
                if neighbor_tile in visited:
                    # print("%s has already been visited")
                    continue
                (r,c) = neighbor_tile
                idx = r*self.width + c
                if self.MAP[idx] == 100:
                    # print("(%s, %s) is an obstacle"%(r,c))
                    continue
                q0 = (rr,cc,current_theta)
                q1 = (r,c,theta)
                path = dubins.shortest_path(q0, q1, self.turning_radius)
                # print("DUBIN PATH FROM %s to %s -> %s"%(q0,q1,path))
                # configurations, _ = path.sample_many(2)
                # print("CONFIGS: %s"%(configurations))
                dubin_dist = path.path_length()
                # print("DUBIN DIST: %s"%dubin_dist)
                if self.MAP[idx] == 0 and distance[neighbor_tile] > distance[current_tile] + dubin_dist:
                    parent[neighbor_tile] = current_tile
                    distance[neighbor_tile] = distance[current_tile] + dubin_dist
                    q.put((neighbor_tile,theta))

            if q.empty():
                print("QUEUEUEUEUEUEUE IS EMPTY!!!! IT SHOULD NOT BE YOU DUBASS!")

            visited.add(current_tile)
        path = []
        path.append(destination)
        if destination is None:
            return None
        while parent[destination] != destination:
            destination = parent[destination]
            path.append(destination)
        path_in_meters = []
        path = reversed(path)
        for p in path:
            (r,c) = p
            path_in_meters.append(self.to_meter_coords((r+0.5,c+0.5)))
        self.publish_map()
        return path_in_meters

    def get_out(self):
        print("STARTING")
        # while self.robot.dist_from_start < self.R_MAZE:
            #dance a little bit
        while True:
            print("saving map...")
            self.save_map()
            print("getting path...")
            path = self.get_exploration_path()
            self.trajectory = LineTrajectory('/built_trajectory')
            self.data_points = []

            if path is None:
                continue
            for p in path:
                point = Point()
                point.x = p[0]
                point.y = p[1]
                self.trajectory.addPoint(point)
                self.data_points.append(point)
            
            # self.trajectory.publish_trajectory()
            print("publishing trajectory")
            self.publish_trajectory()
            self.mark_pt(self.trajectory_points, (1.0,0,0), self.data_points)
            # print("PATH: ",path)
            self.publish_nodes(path)

MazeSolver()
