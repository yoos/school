#!/usr/bin/env python2

import sys
import csv

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



print startState


