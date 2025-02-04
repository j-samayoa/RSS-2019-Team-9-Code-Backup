import rospy
import numpy as np
from yaml import load
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3, Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped, Polygon, Point32, PoseWithCovarianceStamped, PointStamped
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import OccupancyGrid
import json
import tf.transformations
import tf
from scipy import ndimage
import math


EPSILON = 0.00000000001

''' These data structures can be used in the search function
'''


def ceiling(n):
    floor = float(int(n))
    if n - floor > 0:
        return int(floor) + 1
    else:
        return int(floor)

def frange(x, y, jump):
  while x < y:
    yield x
    x += jump

def dist(point1,point2):
    (x1,y1) = point1
    (x2,y2) = point2
    return ((y2-y1)**2 + (x2-x1)**2)**0.5

def load_params_from_yaml(fp):
    with open(fp, 'r') as infile:
        yaml_data = load(infile)
        for param in yaml_data:
            print "param:", param, ":", yaml_data[param]
            rospy.set_param("~"+param, yaml_data[param])

def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy   
    
def rviz_to_pixels(self,p):
    origin = (25.9,48.5)
    theta = -3.14
    resolution = 0.0504
    height = 1300
    x,y = rotate(origin,p,theta)

    x = x - origin[0]
    y = y - origin[1]
    
    yy = x/resolution
    xx = height - y/resolution
    return (xx,yy)

def pixels_to_rviz(self,p):
    origin = (25.9,48.5)
    theta = 3.14
    resolution = 0.0504
    height = 1300
    y = (height - p[0])*resolution
    x = p[1] * resolution


    x = origin[0] + x
    y = origin[1] + y

    (xx,yy) = rotate(origin,(x,y),theta)

    return (xx,yy)

class LineTrajectory(object):
    """ A class to wrap and work with piecewise linear trajectories. """
    def __init__(self, viz_namespace=None):
        self.points = []
        self.np_points = None
        self.distances = []
        self.speed_profile = []
        self.has_acceleration = False
        self.visualize = False
        self.viz_namespace = viz_namespace
        self.speed_interpolater = None

        if viz_namespace:
            self.visualize = True
            self.start_pub = rospy.Publisher(viz_namespace + "/start_point", Marker, queue_size = 1)
            self.traj_pub  = rospy.Publisher(viz_namespace + "/path", Marker, queue_size = 1)
            self.end_pub   = rospy.Publisher(viz_namespace + "/end_pose", Marker, queue_size = 1)
            self.speed_pub   = rospy.Publisher(viz_namespace + "/speeds", MarkerArray, queue_size = 1)

    # compute the distances along the path for all path segments beyond those already computed
    def update_distances(self):
        num_distances = len(self.distances)
        num_points = len(self.points)

        for i in xrange(num_distances,num_points):
            if i == 0:
                self.distances.append(0)
            else:
                p0 = self.points[i-1]
                p1 = self.points[i]
                delta = np.array([p0[0]-p1[0],p0[1]-p1[1]])
                self.distances.append(self.distances[i-1] + np.linalg.norm(delta))

    def distance_to_end(self, t):
        if not len(self.points) == len(self.distances):
            print "WARNING: Different number of distances and points, this should never happen! Expect incorrect results. See LineTrajectory class."
        dat = self.distance_along_trajectory(t)
        if dat == None:
            return None
        else:
            return self.distances[-1] - dat

    def distance_along_trajectory(self, t):
        # compute distance along path
        # ensure path boundaries are respected
        if t < 0 or t > len(self.points) - 1.0:
            return None
        i = int(t) # which segment
        t = t % 1.0 # how far along segment
        if t < EPSILON:
            return self.distances[i]
        else:
            return (1.0-t)*self.distances[i] + t*self.distances[i+1]

    def addPoint(self, point):
        print "adding point to trajectory:", point.x, point.y
        self.points.append((point.x, point.y))
        self.update_distances()
        self.mark_dirty()

    def clear(self):
        self.points = []
        self.distances = []
        self.speed_profile = []
        self.speed_interpolater = None
        self.mark_dirty()

    def empty(self):
        return len(self.points) == 0

    def save(self, path):
        print "Saving trajectory to:", path
        data = {}
        data["points"] = []
        for p in self.points:
            data["points"].append({"x": p[0], "y": p[1]})
        with open(path, 'w') as outfile:
            json.dump(data, outfile)

    def mark_dirty(self):
        self.has_acceleration = False

    def dirty(self):
        return not self.has_acceleration

    def load(self, path):
        print "Loading trajectory:", path
        with open(path) as json_file:
            json_data = json.load(json_file)
            for p in json_data["points"]:
                self.points.append((p["x"], p["y"]))
        self.update_distances()
        print "Loaded:", len(self.points), "points"
        self.mark_dirty()

    # put the points into a KD tree for faster nearest neighbors queries
    def make_np_array(self):
        self.np_points = np.array(self.points)
        self.np_distances = np.array(self.distances)
        self.has_acceleration = True

    # build a trajectory class instance from a trajectory message
    def fromPolygon(self, trajMsg):
        for p in trajMsg.points:
            self.points.append((p.x, p.y))
            if p.z >= 0:
                self.speed_profile.append(p.z)
        self.update_distances()
        self.mark_dirty()
        print "Loaded new trajectory with:", len(self.points), "points"

    def toPolygon(self):
        poly = PolygonStamped()
        poly.header = self.make_header("/cartographer_map")
        use_speed_profile = len(self.speed_profile) == len(self.points)
        for i in xrange(len(self.points)):
            p = self.points[i]
            pt = Point32()
            pt.x = p[0]
            pt.y = p[1]
            if use_speed_profile:
                pt.z = self.speed_profile[i]
            else:
                pt.z = -1
            poly.polygon.points.append(pt)
        return poly

    def publish_start_point(self, duration=0.0, scale=0.1):
        should_publish = len(self.points) > 0
        if self.visualize and self.start_pub.get_num_connections() > 0:
            print "Publishing speed profile"
            marker = Marker()
            marker.header = self.make_header("/cartographer_map")
            marker.ns = self.viz_namespace + "/trajectory"
            marker.id = 0
            marker.type = 2 # sphere
            marker.lifetime = rospy.Duration.from_sec(duration)
            if should_publish:
                marker.action = 0
                marker.pose.position.x = self.points[0][0]
                marker.pose.position.y = self.points[0][1]
                marker.pose.orientation.w = 1.0
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                # delete marker
                marker.action = 2

            self.start_pub.publish(marker)
        # elif self.start_pub.get_num_connections() == 0:
        #   print "Not publishing start point, no subscribers"

    def publish_end_point(self, duration=0.0):
        should_publish = len(self.points) > 1
        if self.visualize and self.end_pub.get_num_connections() > 0:
            print "Publishing end point"
            marker = Marker()
            marker.header = self.make_header("/cartographer_map")
            marker.ns = self.viz_namespace + "/trajectory"
            marker.id = 1
            marker.type = 2 # sphere
            marker.lifetime = rospy.Duration.from_sec(duration)
            if should_publish:
                marker.action = 0
                marker.pose.position.x = self.points[-1][0]
                marker.pose.position.y = self.points[-1][1]
                marker.pose.orientation.w = 1.0
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                # delete marker
                marker.action = 2

            self.end_pub.publish(marker)
        # elif self.end_pub.get_num_connections() == 0:
        #   print "Not publishing end point, no subscribers"

    def publish_trajectory(self, duration=0.0):
        should_publish = len(self.points) > 1
        if self.visualize and self.traj_pub.get_num_connections() > 0:
            print ("Publishing trajectory")
            marker = Marker()
            marker.header = self.make_header("/cartographer_map")
            marker.ns = self.viz_namespace + "/trajectory"
            marker.id = 2
            marker.type = 4 # line strip
            marker.lifetime = rospy.Duration.from_sec(duration)
            if should_publish:
                marker.action = 0
                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.05
                marker.color.r = 0.6
                marker.color.g = 0
                marker.color.b = 0
                marker.color.a = 1.0
                for p in self.points:
                    pt = Point32()
                    pt.x = p[0]
                    pt.y = p[1]
                    pt.z = 1
                    marker.points.append(pt)
            else:
                # delete
                marker.action = 2
            self.traj_pub.publish(marker)
        elif self.traj_pub.get_num_connections() == 0:
            print ("Not publishing trajectory, no subscribers")

    def publish_speeds(self, duration=0.0, scale=0.7):
        should_publish = len(self.speed_profile) > 1
        if self.visualize and self.speed_pub.get_num_connections() > 0:
            if self.dirty():
                self.make_np_array()
            markers = [marker_clear_all("/cartographer_map")]
            normed_speeds = np.array(self.speed_profile) / np.max(self.speed_profile)
            last_speed = 0.0
            for i, speed in enumerate(normed_speeds):
                if speed >= last_speed * 0.99:
                    color = ColorRGBA(0, 1, 0, 0.8)
                else:
                    color = ColorRGBA(1, 0, 0, 0.8)
                last_speed = speed
                markers.append(marker_from_point_radius(self.np_points[i,:], np.power(speed, 0.8) * scale,
                    index=i, linewidth=0.05, color=color, lifetime=duration))

            marker_array = MarkerArray(markers=markers)
            self.speed_pub.publish(marker_array)

    def speed_at_t(self, t):
        if self.speed_interpolater == None:
            self.xs = np.arange(len(self.speed_profile))
            self.ys = self.speed_profile
            self.speed_interpolater = lambda x: np.interp(x,  self.xs, self.ys)

        return self.speed_interpolater(t)

    def publish_viz(self, duration=0):
        if not self.visualize:
            print ("Cannot visualize path, not initialized with visualization enabled")
            return

        self.publish_start_point(duration=duration)
        self.publish_trajectory(duration=duration)
        self.publish_end_point(duration=duration)
        #self.publish_speeds(duration=duration)

    def make_header(self, frame_id, stamp=None):
        if stamp == None:
            stamp = rospy.Time.now()
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        return header
