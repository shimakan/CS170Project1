import numpy as np
from math import sqrt, ceil, floor

class Node:
    def __init__(self, BoardState, Parent, Depth, PathCost, HeuristicCost):
        self.BoardState = BoardState
        self.Parent = Parent
        self.Depth = Depth
        self.PathCost = PathCost
        self.HeuristicCost = HeuristicCost
        
        # Children Nodes
        self.left  = None
        self.down = None
        self.up = None
        self.right = None

    def MoveLeft(self):
        # returns the resulting board state after moving the blank node left
        # index of the empty value 
        idx = np.where(self.BoardState == 0)
        if(idx[0] <= 0):
            return None, False
        else:
             # create a new state with the 0 value moved
             leftval = self.BoardState[idx[0], idx[1] - 1]
             child = np.copy(self.BoardState)
             # switch the values 
             child[idx[0], idx[1] - 1] = 0
             child[idx[0], idx[1]] = leftval
             return child, True

    def MoveDown(self):
        # returns the resulting board state after moving the blank node down
        # index of the empty value 
        idx = np.where(self.BoardState == 0)
        if(idx[0] >= self.BoardState.shape[0] - 1):
            return None, False
        else:
             # create a new state with the 0 value moved
             downval = self.BoardState[idx[0] + 1, idx[1]]
             child = np.copy(self.BoardState)
             child[idx[0] + 1, idx[1]] = 0
             child[idx[0], idx[1]] = downval
             return child, True

    def MoveUp(self):
        # returns the resulting board state after moving the blank node up
        # index of the empty value 
        idx = np.where(self.BoardState == 0)
        if(idx[0] <= 0):
            return None, False
        else:
             # create a new state with the 0 value moved
             upval = self.BoardState[idx[0] - 1, idx[1]]
             child = np.copy(self.BoardState)
             child[idx[0] - 1, idx[1]] = 0
             child[idx[0], idx[1]] = upval
             return child, True

    def MoveRight(self):
        # returns the resulting board state after moving the blank node right
        # index of the empty value 
        idx = np.where(self.BoardState == 0)
        if(idx[1] >= self.BoardState.shape[0] - 1):
            # print(self.BoardState.shape[0] - 1)
            return None, False
        else:
             # create a new state with the 0 value moved
             rightval = self.BoardState[idx[0], idx[1] + 1]
             child = np.copy(self.BoardState)
             child[idx[0], idx[1] + 1] = 0
             child[idx[0], idx[1]] = rightval
             return child, True
        
    def GoalTest(self,GoalState): 
        return np.array_equal(self.BoardState, GoalState)

    #def QueueingFunction(node):
        # chooses the search method?
        pass

    def GeneralSearch(self, QueueingFunction, GoalState):
        nodes = [(self,0)] # queue of nodes
        visited = set([]) # for avoiding duplicate states
        depthq = [(0,0)] # queue of all the depths
        pathcostq = [0] # queue of all the path costs

        while(nodes):
            node = nodes.pop()[0]
            visited.add(tuple(node.BoardState.flatten()))

            if(node.GoalTest(GoalState)):
                return node
            nodes, depthq, pathcostq = QueueingFunction(nodes, node, visited, depthq, pathcostq) 
            # ... 

        # if the queue is empty, no solution was found 
        return "failure"

    def UniformCost(self, nodes, node, visited, depthq, pathcostq):
        # uniform cost search
        # enqueue nodes in order of cumulative cost
        depth = depthq.pop()[0]
        cost = pathcostq.pop()
        # Expand node
        # check validity left
        if(node.MoveLeft()[1]):
            state = node.MoveLeft()[0]
            if tuple(state.flatten()) not in visited: 
                node.left = Node(BoardState = state, Parent = node,\
                                 Depth = depth + 1, PathCost = cost + 1, HeuristicCost = 0)
                nodes.append((node.left, cost + 1))
                depthq.append((depth + 1, cost + 1))
                pathcostq.append(cost + 1)
        # check validity of down
        if(node.MoveDown()[1]):
            state = node.MoveDown()[0]
            if tuple(state.flatten()) not in visited: 
                node.down = Node(BoardState = state, Parent = node,\
                                 Depth = depth + 1, PathCost = cost + 1, HeuristicCost = 0)
                nodes.append((node.down, cost + 1))
                depthq.append((depth + 1, cost + 1))
                pathcostq.append(cost + 1)
        # check validity of up
        if(node.MoveUp()[1]):
            state = node.MoveUp()[0]
            if tuple(state.flatten()) not in visited: 
                node.up = Node(BoardState = state, Parent = node,\
                                 Depth = depth + 1, PathCost = cost + 1, HeuristicCost = 0)
                nodes.append((node.up, cost + 1))
                depthq.append((depth + 1, cost + 1))
                pathcostq.append(cost + 1)
        # check validity right
        if(node.MoveRight()[1]):
            state = node.MoveRight()[0]
            if tuple(state.flatten()) not in visited: 
                node.right = Node(BoardState = state, Parent = node,\
                                 Depth = depth + 1, PathCost = cost + 1, HeuristicCost = 0)
                nodes.append((node.right, cost + 1))
                depthq.append((depth + 1, cost + 1))
                pathcostq.append(cost + 1)

        nodes = sorted(nodes, key=lambda x:x[1], reverse = True)
        depthq = sorted(depthq, key=lambda x:x[1], reverse = True)
        pathcostq = sorted(pathcostq, key=lambda x:x, reverse = True)
        return nodes, depthq, pathcostq

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