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
actionSeq      = []

startFile = open(startFileName, 'rU')
startState = [[int(num) for num in bank] for bank in list(csv.reader(startFile, delimiter=','))]
startFile.close

goalFile = open(goalFileName, 'rU')
goalState = [[int(num) for num in bank] for bank in list(csv.reader(goalFile, delimiter=','))]
goalFile.close


endState = actions.act1(startState)


actionSeq.append(startState)
actionSeq.append(endState)

print actionSeq


outputFile = open(outputFileName, 'wb')
writer = csv.writer(outputFile, delimiter=',')
writer.writerows(actionSeq)
outputFile.close()

