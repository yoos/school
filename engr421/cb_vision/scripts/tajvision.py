#!/usr/bin/env python2

import sys
import cv2
import time
import numpy
import os

# Set up node for ROS.
import roslib; roslib.load_manifest("cb_vision")
import rospy

##
# Opens a video capture device with a resolution of 800x600
# at 30 FPS.
##
def open_camera(cam_id = 0):
    cap = cv2.VideoCapture(cam_id)
    cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 240);
    cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 320);
    #cap.set(cv2.cv.CV_CAP_PROP_FPS, 30);
    return cap

##
# Gets a frame from an open video device, or returns None
# if the capture could not be made.
##
def get_frame(device):
    ret, img = device.read()
    if (ret == False): # failed to capture
        print >> sys.stderr, "Error capturing from video device."
        return None
    return img
 
##
# Closes all OpenCV windows and releases video capture device
# before exit.
##
def cleanup(cam_id = 0): 
    cv2.destroyAllWindows()
    cv2.VideoCapture(cam_id).release()
 
##
# Creates a new RGB image of the specified size, initially
# filled with black.
##
def new_rgb_image(width, height):
    image = numpy.zeros( (height, width, 3), numpy.uint8)
    return image

# Global variable containing the 4 points selected by the user in the corners of the board
corner_point_list = []
 
##
# This function is called by OpenCV when the user clicks
# anywhere in a window displaying an image.
##
def mouse_click_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print "Click at (%d,%d)" % (x,y)
        corner_point_list.append( (x,y) )
 
##
# Prompt user to select the four corners of the board.
#
# Parameters:
# * dev: Video Device (from open_camera())
##
def get_board_corners(dev):
    # Read a frame from the video device
    img = get_frame(dev)
 
    # Displace image to user
    cv2.imshow("Calibrate", img)
 
    # Register the mouse callback on this window. When 
    # the user clicks anywhere in the "Calibrate" window,
    # the function mouse_click_callback() is called (defined above)
    cv2.setMouseCallback("Calibrate", mouse_click_callback)
 
    # Wait until the user has selected 4 points
    while True:
        # If the user has selected all 4 points, exit loop.
        if (len(corner_point_list) >= 4):
            print "Got 4 points: "+str(corner_point_list)
            break
 
        # If the user hits a key, exit loop, otherwise remain.
        if (cv2.waitKey(10) >= 0):
            break;
 
    # Close the calibration window:
    cv2.destroyWindow("Calibrate")

    # Put corner coordinates on ROS parameter server.
    rospy.set_param("/cb_board/corner0/x", corner_point_list[0][0])
    rospy.set_param("/cb_board/corner0/y", corner_point_list[0][1])
    rospy.set_param("/cb_board/corner1/x", corner_point_list[1][0])
    rospy.set_param("/cb_board/corner1/y", corner_point_list[1][1])
    rospy.set_param("/cb_board/corner2/x", corner_point_list[2][0])
    rospy.set_param("/cb_board/corner2/y", corner_point_list[2][1])
    rospy.set_param("/cb_board/corner3/x", corner_point_list[3][0])
    rospy.set_param("/cb_board/corner3/y", corner_point_list[3][1])
 

##################################################### 
### Calibration Example ###
if __name__ == "__main__":
    # Initialize ROS node.
    rospy.init_node("cb_rectify", anonymous=False)

    cam_id = 1
    dev = open_camera(cam_id)

    # Get board corners.
    get_board_corners(dev)
 

    # # Size (in pixels) of the transformed image
    # transform_size = (int(board_size[0]*dpi), int(board_size[1]*dpi))
    # while True:
    #     img_orig = get_frame(dev)
    #     if img_orig is not None: # if we did get an image
    #         # Show the original (untransformed) image
    #         cv2.imshow("video", img_orig)
    #
    #         # Apply the transformation matrix to skew the image and display it
    #         img = cv2.warpPerspective(img_orig, transform, dsize=transform_size)
    #         cv2.imshow("warped", img)
 
    #     else: # if we failed to capture (camera disconnected?), then quit
    #         break
 
    #     if (cv2.waitKey(2) >= 0):
    #         break
 
    cleanup(cam_id)
