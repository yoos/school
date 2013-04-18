#!/usr/bin/env python2

import sys
import csv
import actions

if len(sys.argv) < 5:
	print "Usage: " + sys.argv[0] + " [initial state file] [goal state file] [mode] [output file]"
	exit()

startFileName  = sys.argv[1]
goalFileName   = sys.argv[2]
mode           = sys.argv[3]
outputFileName = sys.argv[4]

startFile = open(startFileName, 'rU')
startState = list(csv.reader(startFile, delimiter=','))
startFile.close

goalFile = open(goalFileName, 'rU')
goalState = list(csv.reader(goalFile, delimiter=','))
goalFile.close



print startState


