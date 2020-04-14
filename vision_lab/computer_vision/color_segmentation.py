import cv2
import imutils
import numpy as np
import pdb

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
# 
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def restrict_view(img):
    """
    Will make black bar lines on the top and bottom of the image, restricting
    the cameras view. This is for the orange line follower.
    """
    height = len(img)
    width = len(img[0])
    for i in range(height//4): #bottom black line
        img[i] = [[0,0,0] for j in range(width)]
    for i in range(height//2,height): #top black line
        img[i] = [[0,0,0] for j in range(width)]
    return img

def cd_color_segmentation(img, template):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########
        
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # convert image to HSV color space
        #hsv = cv2.cvtColor(restrict_view(img), cv2.COLOR_BGR2HSV) # convert image to HSV color space

        # define range of orange color in HSV
        upper_orange = np.array([30, 255, 255])
        lower_orange = np.array([7, 190, 190])

        # Threshold the HSV iamge to get only orange colors
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        
        # Bitwise-AND mask and original image if want to see only cone with color
        #res = cv2.bitwise_and(img, img, mask=mask)

        # Erosion and Dilation
        kernel = np.ones((6, 6), np.uint8)
        ero_dil = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        #detected_cone_points = np.where(ero_dil > 0)
        #avg_x = np.mean(detected_cone_points[1])
        #avg_y = np.mean(detected_cone_points[0])

        #std = np.std(detected_cone_points, axis=0)
        #std_x = std[0]
        #std_y = std[1]

        #min_x = None
        #min_y = None
        #max_x = None
        #max_y = None
        #for x in range(len(ero_dil[0])):
        #    for y in range(len(ero_dil)):
        #        if ero_dil[y][x] > 0:
        #            if min_x == None or x < min_x:
        #                if abs(x - avg_x) < 2*std_x:
        #                    min_x = x
        #            if min_y == None or y < min_y:
        #                if abs(y - avg_y) < 2*std_y:
        #                    min_y = y
        #            if max_x == None or x > max_x:
        #                if abs(x - avg_x) < 2*std_x:
        #                    max_x = x
        #            if max_y == None or y > max_y:
        #                if abs(y - avg_y) < 2*std_y:
        #                    max_y = y

        #bounding_box = ((min_x, min_y), (max_x, max_y))
        
        # finding bounding rectangle for cone using erosion/dilation mask
        x, y, w, h = cv2.boundingRect(ero_dil)
        bounding_box = ((x, y), (x+w, y+h))
        
        # drawing the bounding box onto the image
        cv2.rectangle(img, bounding_box[0], bounding_box[1], (255,0,0), 2)
        
        #image_print(mask)
        #image_print(ero_dil)
        image_print(img)

	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box

#cone = cv2.imread('real_cone.png', cv2.IMREAD_COLOR)
#cd_color_segmentation(cone, 'test_images_cone/cone_template.png')
