import copy

def moveBoat(gs):
    gs[0][2] = 1 - gs[0][2]
    gs[1][2] = 1 - gs[1][2]

def getBanks(gs):
    return 1-gs[0][2], gs[0][2]

# The following five functions generate successors for a game state. They are
# numbered according to the order specified in the assignment. Each function
# takes in a game state tuple and outputs the successor, if it exists. If
# a successor does not exist (i.e., the action would be invalid), the function
# outputs an empty list.

# Put one missionary in the boat
def act1(gs):
    succ = copy.deepcopy(gs)

    # The boat is currently at curBank.
    curBank, endBank = getBanks(gs)

    # Move people.
    if succ[curBank][0] >= 1:
        succ[curBank][0] -= 1
        succ[endBank][0] += 1

    # Move boat to other side.
    moveBoat(succ)

    return succ

# Put two missionaries in the boat
def act2(gs):
    curBank, endBank = getBanks(gs)


# Put one cannibal in the boat
def act3(gs):
    curBank, endBank = getBanks(gs)


# Put one cannibal and one missionary in the boat
def act4(gs):
    curBank, endBank = getBanks(gs)


# Put two cannibals in the boat
def act5(gs):
    curBank, endBank = getBanks(gs)





# vim: expandtab

