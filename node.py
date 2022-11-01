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
        if(idx[1] <= 0):
            # not possible to move left
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
            # not possible to move down
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
            # not possible to move up
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
            # not possible to move right
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

    def GeneralSearch(self, QueueingFunction, GoalState):
        nodes = [(self,0)] # queue of nodes
        visited = set([]) # for avoiding duplicate states
        depthq = [(0,0)] # queue of all the depths
        pathcostq = [(0,0)] # queue of all the path costs
        nodesExpanded = 0

        idxs = {} # dictionary {key = int : value = (x,y) coordinate of val in goal array}
                  # used in manhattan distance heuristic
        for i in range(np.shape(GoalState)[0]):
            for j in range(np.shape(GoalState)[1]):
                idxs[GoalState[i, j]] = tuple([i,j])

        while(nodes):
            node = nodes.pop()[0] 
            #print(f"Best Node: h(n): {node.HeuristicCost}, g(n): {node.PathCost} \n {node.BoardState} \n")
            visited.add(tuple(node.BoardState.flatten()))

            if(node.GoalTest(GoalState) == True):
                print(f"Depth: {node.Depth}, Nodes Expanded: {nodesExpanded}")
                return node.BoardState
            nodesExpanded += 1
            nodes, depthq, pathcostq = QueueingFunction(nodes, node, visited, depthq, pathcostq, GoalState, idxs) 
        # if the queue is empty, no solution was found 
        return "failure"

    def UniformCost(self, nodes, node, visited, depthq, pathcostq, goal, idxs):
        # uniform cost search
        # enqueue nodes children of expanded nodes in order of cumulative cost
        depth = depthq.pop()[0]
        cost = pathcostq.pop()[0]
        # check validity left
        if(node.MoveLeft()[1]):
            state = node.MoveLeft()[0]
            if tuple(state.flatten()) not in visited: 
                node.left = Node(BoardState = state, Parent = node,\
                                 Depth = depth + 1, PathCost = cost + 1, HeuristicCost = 0)
                nodes.append((node.left, cost + 1,depth+1))
                depthq.append((depth + 1, cost + 1))
                pathcostq.append((cost + 1,depth+1))
        # check validity of down
        if(node.MoveDown()[1]):
            state = node.MoveDown()[0]
            if tuple(state.flatten()) not in visited: 
                node.down = Node(BoardState = state, Parent = node,\
                                 Depth = depth + 1, PathCost = cost + 1, HeuristicCost = 0)
                nodes.append((node.down, cost + 1,depth+1))
                depthq.append((depth + 1, cost + 1))
                pathcostq.append((cost + 1,depth+1))
        # check validity of up
        if(node.MoveUp()[1]):
            state = node.MoveUp()[0]
            if tuple(state.flatten()) not in visited: 
                node.up = Node(BoardState = state, Parent = node,\
                                 Depth = depth + 1, PathCost = cost + 1, HeuristicCost = 0)
                nodes.append((node.up, cost + 1,depth+1))
                depthq.append((depth + 1, cost + 1))
                pathcostq.append((cost + 1,depth+1))
        # check validity right
        if(node.MoveRight()[1]):
            state = node.MoveRight()[0]
            if tuple(state.flatten()) not in visited: 
                node.right = Node(BoardState = state, Parent = node,\
                                 Depth = depth + 1, PathCost = cost + 1, HeuristicCost = 0)
                nodes.append((node.right, cost + 1,depth+1))
                depthq.append((depth + 1, cost + 1))
                pathcostq.append((cost + 1,depth+1))

        nodes = sorted(nodes, key=lambda x:(x[1], x[2]), reverse = True)
        depthq = sorted(depthq, key=lambda x:(x[1], x[0]), reverse = True)
        pathcostq = sorted(pathcostq, key=lambda x:(x[0], x[1]), reverse = True)
        return nodes, depthq, pathcostq

    def MisplacedTile(self, nodes, node, visited, depthq, pathcostq, goal, idxs):
    # f(n) = g(n) + h(n)
    # g(n) is a function of depth
    # h(n) is the sum of all misplaced tiles
    # enqueue children of expanded nodes in terms of f(n)
        depth = depthq.pop()[0]
        cost = pathcostq.pop()
        if(node.MoveLeft()[1]):
            state = node.MoveLeft()[0]
            if tuple(state.flatten()) not in visited:
                misplaced = np.sum(state != goal)
                node.left = Node(BoardState = state, Parent = node,\
                                 Depth = depth + 1, PathCost = ((depth + 1) + misplaced), HeuristicCost = misplaced)
                nodes.append((node.left, node.PathCost, depth+1))
                depthq.append((depth + 1, node.PathCost))
                pathcostq.append((node.PathCost, depth + 1))
        # check validity of down
        if(node.MoveDown()[1]):
            state = node.MoveDown()[0]
            if tuple(state.flatten()) not in visited:
                misplaced = np.sum(state != goal)
                node.down = Node(BoardState = state, Parent = node,\
                                 Depth = depth + 1, PathCost = ((depth + 1) + misplaced), HeuristicCost = misplaced)
                nodes.append((node.down, node.PathCost, depth+1))
                depthq.append((depth + 1, node.PathCost))
                pathcostq.append((node.PathCost,depth+1))
        # check validity of up
        if(node.MoveUp()[1]):
            state = node.MoveUp()[0]
            if tuple(state.flatten()) not in visited:
                misplaced = np.sum(state != goal)
                node.up = Node(BoardState = state, Parent = node,\
                                 Depth = depth + 1, PathCost = ((depth + 1) + misplaced), HeuristicCost = misplaced)
                nodes.append((node.up, node.PathCost,depth+1))
                depthq.append((depth + 1, node.PathCost))
                pathcostq.append((node.PathCost,depth+1))
        # check validity right
        if(node.MoveRight()[1]):
            state = node.MoveRight()[0]
            if tuple(state.flatten()) not in visited:
                misplaced = np.sum(state != goal)
                node.right = Node(BoardState = state, Parent = node,\
                                 Depth = depth + 1, PathCost = ((depth + 1) + misplaced), HeuristicCost = misplaced)
                nodes.append((node.right, node.PathCost,depth+1))
                depthq.append((depth + 1, node.PathCost))
                pathcostq.append((node.PathCost,depth+1))

        nodes = sorted(nodes, key=lambda x:(x[1],x[2]), reverse = True)
        depthq = sorted(depthq, key=lambda x:(x[1],x[0]), reverse = True)
        pathcostq = sorted(pathcostq, key=lambda x:(x[0],x[1]), reverse = True)
        return nodes, depthq, pathcostq

    def Manhattan(self, nodes, node, visited, depthq, pathcostq, goal, idxs):
        depth = depthq.pop()[0]
        cost = pathcostq.pop()
        if(node.MoveLeft()[1]):
            state = node.MoveLeft()[0]
            if tuple(state.flatten()) not in visited:
                mandist = man(state, goal, idxs)
                node.left = Node(BoardState = state, Parent = node,\
                                 Depth = depth + 1, PathCost = ((depth + 1) + mandist), HeuristicCost = mandist)
                nodes.append((node.left, node.PathCost, depth + 1))
                depthq.append((depth + 1, node.PathCost))
                pathcostq.append((node.PathCost, depth + 1))
        # check validity of down
        if(node.MoveDown()[1]):
            state = node.MoveDown()[0]
            if tuple(state.flatten()) not in visited:
                mandist = man(state, goal, idxs)
                node.down = Node(BoardState = state, Parent = node,\
                                 Depth = depth + 1, PathCost = ((depth + 1) + mandist), HeuristicCost = mandist)
                nodes.append((node.down, node.PathCost, depth + 1))
                depthq.append((depth + 1, node.PathCost))
                pathcostq.append((node.PathCost, depth + 1))
        # check validity of up
        if(node.MoveUp()[1]):
            state = node.MoveUp()[0]
            if tuple(state.flatten()) not in visited:
                mandist = man(state, goal, idxs)
                node.up = Node(BoardState = state, Parent = node,\
                                 Depth = depth + 1, PathCost = ((depth + 1) + mandist), HeuristicCost = mandist)
                nodes.append((node.up, node.PathCost, depth + 1))
                depthq.append((depth + 1, node.PathCost))
                pathcostq.append((node.PathCost, depth + 1))
        # check validity right
        if(node.MoveRight()[1]):
            state = node.MoveRight()[0]
            if tuple(state.flatten()) not in visited:
                mandist = man(state, goal, idxs)
                node.right = Node(BoardState = state, Parent = node,\
                                 Depth = depth + 1, PathCost = ((depth + 1) + mandist), HeuristicCost = mandist)
                nodes.append((node.right, node.PathCost, depth+1))
                depthq.append((depth + 1, node.PathCost))
                pathcostq.append((node.PathCost, depth+1))

        nodes = sorted(nodes, key=lambda x:(x[1], x[2]), reverse = True)
        depthq = sorted(depthq, key=lambda x:(x[1], x[0]), reverse = True)
        pathcostq = sorted(pathcostq, key=lambda x:(x[0], x[1]), reverse = True)
        return nodes, depthq, pathcostq
                
def man(arr, goal, idxs):
    # computes the manhattan distance between the provided array and the goal array
    # idxs is a dictionary key = int, value = coordinate in goal array
    msum = 0
    for i in range(np.shape(goal)[0]):
        for j in range(np.shape(goal)[1]):
            if(arr[i,j] != 0):
                msum += abs((i - idxs[arr[i,j]][0])) + abs((j - idxs[arr[i,j]][1]))
    return msum

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