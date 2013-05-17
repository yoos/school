#!/usr/bin/env python2

import sys
import cv2
import time
import numpy
import os

##
# Opens a video capture device with a resolution of 800x600
# at 30 FPS.
##
def open_camera(cam_id = 0):
    cap = cv2.VideoCapture(cam_id)
    cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 600);
    cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 800);
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
# Computes a perspective transform matrix by capturing a single
# frame from a video source and displaying it to the user for
# corner selection.
#
# Parameters:
# * dev: Video Device (from open_camera())
# * board_size: A tuple/list with 2 elements containing the width and height (respectively) of the gameboard (in arbitrary units, like inches)
# * dpi: Scaling factor for elements of board_size
# * calib_file: Optional. If specified, the perspective transform matrix is saved under this filename.
#   This file can be loaded later to bypass the calibration step (assuming nothing has moved).
##
def get_transform_matrix(dev, board_size, dpi, calib_file = None):
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
 
    # If the user selected 4 points
    if (len(corner_point_list) >= 4):
        # Do calibration
 
        # src is a list of 4 points on the original image selected by the user
        # in the order [TOP_LEFT, BOTTOM_LEFT, TOP_RIGHT, BOTTOM_RIGHT]
        src = numpy.array(corner_point_list, numpy.float32)
 
        # dest is a list of where these 4 points should be located on the
        # rectangular board (in the same order):
        dest = numpy.array( [ (0, 0), (0, board_size[1]*dpi), (board_size[0]*dpi, 0), (board_size[0]*dpi, board_size[1]*dpi) ], numpy.float32)
 
        # Calculate the perspective transform matrix
        trans = cv2.getPerspectiveTransform(src, dest)
 
        # If we were given a calibration filename, save this matrix to a file
        if calib_file:
            numpy.savetxt(calib_file, trans)
        return trans
    else:
        return None
 


##################################################### 
### Calibration Example ###
if __name__ == "__main__":
    cam_id = 0
    dev = open_camera(cam_id)
 
    # The size of the board in inches, measured between the two
    # robot boundaries:
    board_size = [22.3125, 45]
 
    # Number of pixels to display per inch in the final transformed image. This
    # was selected somewhat arbitrarily (I chose 17 because it fit on my screen):
    dpi = 17
 
    # Calculate the perspective transform matrix
    transform = get_transform_matrix(dev, board_size, dpi)
 
 
    # Size (in pixels) of the transformed image
    transform_size = (int(board_size[0]*dpi), int(board_size[1]*dpi))
    while True:
        img_orig = get_frame(dev)
        if img_orig is not None: # if we did get an image
            # Show the original (untransformed) image
            cv2.imshow("video", img_orig)
            
            # Apply the transformation matrix to skew the image and display it
            img = cv2.warpPerspective(img_orig, transform, dsize=transform_size)
            cv2.imshow("warped", img)
 
        else: # if we failed to capture (camera disconnected?), then quit
            break
 
        if (cv2.waitKey(2) >= 0):
            break
 
    cleanup(cam_id)
