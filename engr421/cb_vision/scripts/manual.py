#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
from time import sleep

# Set up node for ROS.
import roslib; roslib.load_manifest("cb_vision")
import rospy
from cb_vision.msg import cb_puck_coordinates

#serialPort = '/dev/ttyUSB0'
baudrate = '460800'
header = chr(255)
newlineChar = ''

cmd = 0.0

# Serial write.
def serWrite(myStr):
    try:
        for i in range(len(myStr)):
            ser.write(myStr[i])
    except:
        print "Unable to send data. Check connection."

def callback(pc):
    global cmd
    cmd = pc.x[0]   # Target first puck.

if __name__ == "__main__":
    global cmd
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
        ser = serial.Serial(serialPort, baudrate, timeout=0)
    except serial.SerialException:
        print "Unable to open specified serial port! Exiting..."
        exit(1)
    except NameError:
        for i in range(4):
            try:
                ser = serial.Serial("/dev/ttyUSB"+str(i), baudrate, timeout=0)
                print "Opened serial port at /dev/ttyUSB%d.", i
                break
            except serial.SerialException:
                print "No serial at /dev/ttyUSB%d.", i
                if i == 3:
                    print "No serial found. Giving up!"
                    exit(1)

    sleep(1)

    while True:
        cmd = raw_input("Command to send: ")

        if cmd == 'q':
            exit(0)

        for each in cmd.split(' '):
            serWrite(chr(int(each)))

        sleep(0.005)

# vim: noexpandtab

