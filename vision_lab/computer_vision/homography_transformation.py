import math
import numpy as np
import rospy
import imutils
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge

class Homography():
    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber("/zed/rgb/image_rect_color_mouse_left",Point,self.callback)
        self.marker=rospy.Publisher("xy_marker",Marker,queue_size=1)
        self.robot_marker=rospy.Publisher("robo_marker",Marker,queue_size=1)

    def rviz_mark(self,x,y):
        m = Marker()
        m.color.g=1
        m.pose.orientation.w = 1.0
        m.pose.position.x = x+0.25
        m.pose.position.y = y+0.1
        m.scale.x, m.scale.y, m.scale.z = 0.2, 0.2, 0.2
        self.marker.publish(m)

    def callback(self,data):
        #img_rot=imutils(data,180)

        im_size=np.array([1280.0,720.0,2.0])

        #pts_dst=np.array([[1191.0,34.0,1.0],[1065.0,95.0,1.0],[1026.0,111.0,1.0],[960.0,142.0,1.0],[601.0,160.0,1.0],[597.0,130.0,1.0],[594.0,115.0,1.0],[587.0,60.0,1.0]])
        #pts_dst=np.subtract(im_size,pts_dst_unrot)
        #cv_image=self.bridge.imgmsg_to_cv2(img_rot,desired_encoding="passthrough")

        #pts_src=np.array([[0.252,0.2585,1.0],[0.252,0.3335,1.0],[0.252,0.358,1.0],[0.252,0.433,1.0],[0.024,0.433,1.0],[0.024,0.358,1.0],[0.024,0.3335,1.0],[0.024,0.2585,1.0]])
        
        pts_dst = np.array([[520.0, 18.0, 1.0], [475.0, 53.0, 1.0], [463.0, 61.0, 1.0], [439.0, 79.0, 1.0], [184.0, 39.0, 1.0], [218.0, 66.0, 1.0], [229.0, 73.0, 1.0], [249.0, 88.0, 1.0], [336.0, 90.0, 1.0], [506.0, 82.0, 1.0], [420.0, 114.0, 1.0], [252.0, 121.0, 1.0], [334.0, 127.0, 1.0]])
        pts_src = np.array([[0.22, .113, 1.0], [.295, .113, 1.0], [.32, .113, 1.0], [.395, .113, 1.0], [.22, -.113, 1.0], [.295, -.113, 1.0], [.32, -.113, 1.0], [.395, -.113, 1.0], [.60, 0.0, 1.0], [.60, .30, 1.0], [1.20, .3, 1.0], [1.2, -.3, 1.0], [1.8, 0.0, 1.0]])

        h = cv2.findHomography(pts_dst, pts_src)[0]
        print(h)
        image_pt=np.array([[data.x,data.y,1]]).T
        xy_pt1=np.matmul(h,image_pt)
        xy_pt=xy_pt1/xy_pt1[2]
        self.rviz_mark(xy_pt[0],xy_pt[1])
        print(xy_pt)
if __name__=='__main__':
    try:
        rospy.init_node('HomographyTrans', anonymous=True)
        Homography()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 

