#!/usr/bin/env python2

import sys
import csv
import boat

if len(sys.argv) < 5:
    print "Usage: " + sys.argv[0] + " [initial state file] [goal state file] [mode] [output file]"
    exit()

startFileName  = sys.argv[1]
goalFileName   = sys.argv[2]
searchMode     = sys.argv[3]
outputFileName = sys.argv[4]
expandCounter  = 0
maxDepth       = 0

if searchMode not in ['bfs', 'dfs', 'iddfs', 'astar']:
    print "Mode should be one of:\n    bfs: Breadth-First Search\n    dfs: Depth-First Search\n    iddfs: Iterative-Deepening Depth-First Search\n    astar: A* Search"
    exit()


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
    if gs == goalState_tup:
        return True
    return False


parents = {startState_tup:()}
actions = {startState_tup:()}
depth = {startState_tup:0}

# Expand a node. Returns the set of successors, if any exist.
def expand(gs_tup):
    successors = []

    # Try all five possible actions except the one we've just tried.
    for action in [i for i in range(5) if i != actions[gs_tup]]:
        s_tup = tuplify(boat.act(listify(gs_tup), action))

        # If action was valid and if either the resulting successor is new or
        # its depth is less than that of the same state found previously, add
        # to output lists.
        if s_tup != () and (s_tup not in parents or depth[s_tup] > depth[gs_tup]+1):
            parents.update({s_tup:gs_tup})
            actions.update({s_tup:action})
            depth.update({s_tup:depth[gs_tup]+1})
            successors.append(s_tup)

    return successors


fringe = [startState_tup]

def solution(node):
    path = [node]
    n = node

    while n != startState_tup:
        print "Node:", n, " Parent:", parents[n]
        n = parents[n]
        path.append(n)

    return path

solPath = []

# Graph search. Takes fringe list as input and returns solution path as a list
# (empty if solution does not exist).
def graphSearch(fr):
    global expandCounter, maxDepth
    closed = []

    while True:
        if len(fringe) == 0:
            return []
        if searchMode == 'bfs':
            node = fringe.pop(0)   # Treat list as queue.
        elif searchMode in ['dfs', 'iddfs']:
            node = fringe.pop()   # Treat list as stack.

        expandCounter += 1   # Increment this right after popping from fringe and before expanding.

        if goalTest(node):
            return solution(node)
        if node not in closed:
            closed.append(node)

            successors = expand(node)

            if searchMode == 'iddfs':
                if depth[node] == maxDepth:
                    # Push maxDepth node back onto stack.
                    for succ in successors:
                        fringe.append(succ)
                    successors = []

            for succ in successors:
                print "appending to fringe", succ
                fringe.append(succ)


# Search.
def search():
    global maxDepth
    sol = []

    if searchMode != 'iddfs':
        sol = graphSearch(fringe)
    else:
        while maxDepth < 20:
            maxDepth += 1

            sol = graphSearch(fringe)
            if sol != []:
                return sol

            print maxDepth, sol

    return sol






solPath = search()
print solPath

print "Depth:", depth[goalState_tup]
print "Counter:", expandCounter
print "maxDepth:", maxDepth




outputFile = open(outputFileName, 'wb')
writer = csv.writer(outputFile, delimiter=',')
writer.writerows(solPath)
outputFile.close()



# vim: expandtab

