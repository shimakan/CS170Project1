import numpy as np
from math import sqrt, ceil, floor

class Node:
    def __init__(self, BoardState, Parent, Depth, StepCost, PathCost, HeuristicCost):
        self.BoardState = BoardState
        self.Parent = Parent
        self.Depth = Depth
        self.StepCost = StepCost
        self.PathCost = PathCost
        self.HeuristicCost = HeuristicCost
        
        # Children Nodes
        left  = None
        down = None
        up = None
        right = None

    def MoveLeft(self):
        # returns the resulting board state after moving the blank node left
        # index of the empty value 
        idx = np.where(self.BoardState == 0)
        if(idx[0] <= 0):
            return None
        else:
             # create a new state with the 0 value moved
             leftval = self.BoardState[idx[0], idx[1] - 1]
             child = np.copy(self.BoardState)
             # switch the values 
             child[idx[0], idx[1] - 1] = 0
             child[idx[0], idx[1]] = leftval
        return child

    def MoveDown(self):
        # returns the resulting board state after moving the blank node down
        # index of the empty value 
        idx = np.where(self.BoardState == 0)
        if(idx[0] >= self.BoardState.shape[0] - 1):
            print(self.BoardState.shape[0] - 1)
            return None
        else:
             # create a new state with the 0 value moved
             downval = self.BoardState[idx[0] + 1, idx[1]]
             child = np.copy(self.BoardState)
             child[idx[0] + 1, idx[1]] = 0
             child[idx[0], idx[1]] = downval
        return child

    def MoveUp(self):
        # returns the resulting board state after moving the blank node up
        # index of the empty value 
        idx = np.where(self.BoardState == 0)
        if(idx[0] <= 0):
            return None
        else:
             # create a new state with the 0 value moved
             upval = self.BoardState[idx[0] - 1, idx[1]]
             child = np.copy(self.BoardState)
             child[idx[0] - 1, idx[1]] = 0
             child[idx[0], idx[1]] = upval
        return child

    def MoveRight(self):
        # returns the resulting board state after moving the blank node right
        # index of the empty value 
        idx = np.where(self.BoardState == 0)
        if(idx[1] >= self.BoardState.shape[0] - 1):
            # print(self.BoardState.shape[0] - 1)
            return None
        else:
             # create a new state with the 0 value moved
             rightval = self.BoardState[idx[0], idx[1] + 1]
             child = np.copy(self.BoardState)
             child[idx[0], idx[1] + 1] = 0
             child[idx[0], idx[1]] = rightval
        return child
        
    def GoalTest(self,GoalState): 
        return np.array_equal(self.BoardState, GoalState)

    def QueueingFunction(node):
        # chooses the search method?
        pass

    def GeneralSearch(self, QueueingFunction, GoalState):
        nodes = [self]
        visited = set([]) # for avoiding duplicate states
        depthq = [0] # queue of all the depths
        pathcostq = [0] # queue of all the path costs

        while(nodes):
            node = nodes.pop()
            if(node.GoalTest(GoalState)):
                return node
            # ... 

        # if the queue is empty, no solution was found 
        return "failure"

    def UniformCost(self):
        # uniform cost search
        nodes = [self]
        #GoalState = ...
        pass

def MakeGoal(num):
    #generic goal state maker
    dim = sqrt(num + 1)
    # checking if perfect square
    if ceil(dim) != floor(dim):
        return "unable to make a puzzle of this shape\n"
    
    dim = int(dim)
    GoalState = np.arange(1, num + 2, 1).reshape(dim,dim)
    GoalState[dim - 1, dim - 1] = 0
    return GoalState