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


# Take a list of lists and return a tuple of tuples
def tuplify(lst):
    return tuple(tuple(x) for x in lst)

# Take a tuple of tuples and return a list of lists
def listify(tup):
    return list(list(x) for x in tup)


startFile = open(startFileName, 'rU')
startState = [[int(num) for num in bank] for bank in list(csv.reader(startFile, delimiter=','))]
startState_tup = tuplify(startState)
startFile.close

goalFile = open(goalFileName, 'rU')
goalState = [[int(num) for num in bank] for bank in list(csv.reader(goalFile, delimiter=','))]
goalState_tup = tuplify(goalState)
goalFile.close

print "Start state:", startState


# Return true if game state is goal state. Otherwise, return false.
def goalTest(gs):
    if gs == goalState:
        return True
    return False


parents = {startState_tup:()}
actions = {startState_tup:()}
depth = {startState_tup:0}

# Expand a node. Returns the set of successors, if any exist.
def expand(gs_tup):
    successors = []

    # Try all five possible actions.
    for i in range(5):
        s_tup = tuplify(boat.act(listify(gs_tup), i))

        # If action was valid, add to output list.
        if s_tup != ():
            parents.update({s_tup:gs_tup})
            actions.update({s_tup:i})
            depth.update({s_tup:depth[gs_tup]+1})
            successors.append(s_tup)
            print "Appending", s_tup

    return successors


fringe = [startState_tup]

# Graph search. Takes fringe list as input and returns solution path as a list
# (empty if solution does not exist).
def graphSearch(fr):
    closed = []

    while True:
        if len(fringe) == 0:
            return []
        node = fringe.pop(0)
        if goalTest(node):
            return solution(node)   # TODO
        if node not in closed:
            closed.append(node)
            for succ in expand(node):
                fringe.append(succ)





print expand(tuplify(startState))
print "parents:", parents
print "actions:", actions
print "depth:", depth


print graphSearch(fringe)

print depth[goalState_tup]




outputFile = open(outputFileName, 'wb')
writer = csv.writer(outputFile, delimiter=',')
writer.writerows(actionSeq)
outputFile.close()



# vim: expandtab

