import cv2
import numpy as np

pts_src = np.array([[0.0,0.3048,1.0],[0.0,0.7366,1.0],[0.2794,0.7366,1.0],[0.2794,0.3048,1.0]])
pts_dst = np.array([[525.0,645.0,1.0],[596.0,517.0,1.0],[849.0,502.0,1.0],[1131.0,608.0,1.0]])

h, status = cv2.findHomography(pts_src,pts_dst)
p = np.array([525.0,645.0,1.0])
p = np.transpose(p)
print('homography matrix:', h)
out = np.matmul(h,p)
scalar = out[-1]
out = out/scalar
print('output:', out)
