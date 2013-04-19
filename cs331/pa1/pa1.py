#!/usr/bin/env python2

import sys
import csv
import boat

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

print "Start state:", startState


# Take a list of lists and return a tuple of tuples
def tuplify(lst):
    return tuple(tuple(x) for x in lst)


# Return true if game state is goal state. Otherwise, return false.
def goalTest(gs):
    if gs == goalState:
        return True
    return False


parents = {tuplify(startState):()}
actions = {tuplify(startState):()}
depth = {tuplify(startState):0}

# Expand a node. Returns the set of successors, if any exist.
def expand(gs):
    gs_tup = tuplify(gs)
    successors = []

    # Try all five possible actions.
    for i in range(5):
        s = boat.act(gs, i)
        s_tup = tuplify(s)

        # If action was valid, add to output list.
        if s != []:
            parents.update({s_tup:gs_tup})
            actions.update({s_tup:i})
            depth.update({s_tup:depth[gs_tup]+1})
            successors.append(s_tup)
            print "Appending", s_tup

    return successors





print expand(startState)
print "parents:", parents
print "actions:", actions
print "depth:", depth






outputFile = open(outputFileName, 'wb')
writer = csv.writer(outputFile, delimiter=',')
writer.writerows(actionSeq)
outputFile.close()



# vim: expandtab

