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
startState = tuple([tuple([int(num) for num in bank]) for bank in list(csv.reader(startFile, delimiter=','))])
startFile.close

goalFile = open(goalFileName, 'rU')
goalState = tuple([tuple([int(num) for num in bank]) for bank in list(csv.reader(goalFile, delimiter=','))])
goalFile.close

print "Start state:", startState


# Return true if game state is goal state. Otherwise, return false.
def goalTest(gs):
    if gs == goalState:
        return True
    return False


# Return all possible successors of a node.
def getSuccessors(gs):
    out = []

    # Try all five possible actions.
    for i in range(5):
        succ = actions.act(gs, i)

        # If action was valid, add to output list.
        if succ != []:
            out.append(succ)

    return out

actionSeq.append(startState)
actionSeq.append(endState)

print actionSeq


outputFile = open(outputFileName, 'wb')
writer = csv.writer(outputFile, delimiter=',')
writer.writerows(actionSeq)
outputFile.close()



# vim: expandtab

