#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
from time import sleep

# Set up node for ROS.
import roslib; roslib.load_manifest("cb_vision")
import rospy
from cb_vision.msg import cb_puck_coordinates

import cb_math as m
import cb_config as cfg


puck_count_filtered = [2, cfg.PUCK_COUNT_CONFIRM]   # How many pucks last detected, and how many times we need to confirm.
puck_loc_filtered = [[0.0, 0.0], [0.0, 0.0]]


# Serial write.
def serWrite(myStr):
    try:
        for i in range(len(myStr)):
            ser.write(myStr[i])
    except:
        print "Unable to send data. Check connection."

def callback(pc):
    global puck_loc_filtered
    # Apply calibration offsets to puck locations.
#    pc.x[0] += m.transform((pc.x[0], pc.y[0]), calib_x, calib_y)
    calib_loc = m.transform([[pc.x[0], pc.y[0]], [pc.x[1], pc.y[1]]], cfg.RAIL_OFFSET)

    # Confirm puck count.
    if pc.puck_count != puck_count_filtered[0]:
        if puck_count_filtered[1] > 0:
            puck_count_filtered[1] -= 1
        else:
            puck_count_filtered[0] = pc.puck_count
            puck_count_filtered[1] = cfg.PUCK_COUNT_CONFIRM
    else:
        puck_count_filtered[1] = cfg.PUCK_COUNT_CONFIRM

    # Filter puck locations.
    puck_loc_filtered = m.filter_puck_loc(puck_loc_filtered, calib_loc)

    # Run strategy.
    left, right = cfg.strategy(puck_count_filtered[0], puck_loc_filtered)

    # Send command.
    print "Found %2d pucks at %5f and %5f. Commanding %5f %5f. Filter %d %d" % (pc.puck_count, pc.x[0], pc.x[1], left, right, m.puck_loc_filter_count[0], m.puck_loc_filter_count[1])
    cmd = chr(int(left*127)) + chr(128+int(right*127))
    serWrite(cmd)

# Calibrate puck locations by placing a triangle puck at each corner
# successively. This allows us to be a bit careless with the initial board
# calibration.
calib_x = [0.0] * 4
calib_y = [0.0] * 4
def calibrate_corners():
    global calib_x, calib_y
    for i in range(4):
        calib_x[i] = pc.x[0]
        calib_y[i] = pc.y[0]


if __name__ == "__main__":
    # Initialize ROS node.
    rospy.init_node("cb_comm", anonymous=False)
    rospy.Subscriber("cb_puck_coordinates", cb_puck_coordinates, callback, queue_size=1)

# =========================================================================
    # Try to initialize a serial connection. If serialPort is defined, try
    # opening that. If it is not defined, loop through a range of integers
    # starting from 0 and try to connect to /dev/ttyUSBX where X is the
    # integer. In either case, process dies if serial port cannot be opened.
    #
    # TODO: This needs to be made more concise.
    # =========================================================================
    try:
        ser = serial.Serial(serialPort, cfg.baudrate, timeout=0)
    except serial.SerialException:
        print "Unable to open specified serial port! Exiting..."
        exit(1)
    except NameError:
        for i in range(4):
            try:
                ser = serial.Serial("/dev/ttyUSB"+str(i), cfg.baudrate, timeout=0)
                print "Opened serial port at /dev/ttyUSB%d." % i
                break
            except serial.SerialException:
                print "No serial at /dev/ttyUSB%d." % i
                if i == 3:
                    print "No serial found. Giving up!"
                    exit(1)

#    calibrate_corners()

    rospy.spin()

# vim: expandtab

