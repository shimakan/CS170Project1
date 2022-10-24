import numpy as np
import node
from node import MakeGoal
#from node import *
state = np.array([[1,2,3],
                  [4,8,0],
                  [7,6,5]])

root = node.Node(state, None, 0, 0, 0)
# def __init__(self, BoardState, Parent, Depth, PathCost, HeuristicCost):

# test successive moves (expansion)
# state = root.MoveDown()
# print(state)
# child1 = node.Node(state, root, 0 + 1, 1, 0 + 1, 0)
# 
# state = child1.MoveLeft()
# print(state)
# child2 = node.Node(state, child1, child1.Depth + 1, 1, child1.PathCost + 1, 0)
# 
# state = child2.MoveUp()
# print(state)

# test for GoalTest function
s = np.array([[1,2,3],
              [4,5,6],
              [7,8,0]])
g = MakeGoal(8)
print(root.GeneralSearch(root.UniformCost, g).BoardState)
#print(f"Goal \n {g}\n")
#dummy = node.Node(s, None, 0, 0, 0)
#print(dummy.GoalTest(g))