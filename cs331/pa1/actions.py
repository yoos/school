import copy

def moveBoat(gs):
    gs[0][2] = 1 - gs[0][2]
    gs[1][2] = 1 - gs[1][2]

def getBanks(gs):
    return 1-gs[0][2], gs[0][2]

# Generate a successor given a game state and an action. If the action is
# invalid, the function outputs an empty list.
def act(gs, actNum):
    # Make copy of input list.
    succ = copy.deepcopy(gs)

    # The boat is currently at curBank.
    curBank, endBank = getBanks(gs)

    # Move people. These are numbered according to the order specified in the assignment.
    if actNum == 0:
        # Put one missionary in the boat
        if succ[curBank][0] >= 1:
            succ[curBank][0] -= 1
            succ[endBank][0] += 1
        else:
            return []

    elif actNum == 1:
        # Put two missionaries in the boat
        if succ[curBank][0] >= 2:
            succ[curBank][0] -= 2
            succ[endBank][0] += 2
        else:
            return []

    elif actNum == 2:
        # Put one cannibal in the boat
        if succ[curBank][1] >= 1:
            succ[curBank][1] -= 1
            succ[endBank][1] += 1
        else:
            return []

    elif actNum == 3:
        # Put one cannibal and one missionary in the boat
        if succ[curBank][0] >= 1 and succ[curBank][1] >= 1:
            succ[curBank][0] -= 1
            succ[endBank][0] += 1
            succ[curBank][1] -= 1
            succ[curBank][1] += 1
        else:
            return []

    elif actNum == 4:
        # Put two cannibals in the boat
        if succ[curBank][1] >= 2:
            succ[curBank][1] -= 2
            succ[endBank][1] += 2
        else:
            return []

    else:
        return []

    # Move boat to other side.
    moveBoat(succ)

    return succ


# vim: expandtab

