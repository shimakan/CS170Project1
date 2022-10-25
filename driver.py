import numpy as np
import node
from node import MakeGoal

#state = np.array([[1,2,3],
#                  [4,8,0],
#                  [7,6,5]])

##state = np.array([[8,1,2],
#                  [0,4,3],
#                  [7,6,5]])

state = np.array([[2,1],
                  [3,0]])

root = node.Node(state, None, 0, 0, 0)
# def __init__(self, BoardState, Parent, Depth, PathCost, HeuristicCost):

# test for GoalTest function
s = np.array([[1,2,3],
              [4,5,6],
              [7,8,0]])
g = MakeGoal(3)
print(root.GeneralSearch(root.UniformCost, g))
#print(f"Goal \n {g}\n")
#dummy = node.Node(s, None, 0, 0, 0)
#print(dummy.GoalTest(g))


# Test for misplaced tile
b = np.array([[1,0],
              [3,2]])
