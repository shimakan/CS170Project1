from turtle import up


class Node:
    def __init__(self, BoardState, Parent, Depth, StepCost, PathCost, HeuristicCost):
        BoardState = BoardState  
        Parent = Parent
        Depth = Depth
        StepCost = StepCost
        PathCost = PathCost
        HeuristicCost = HeuristicCost

        # Children Nodes
        left  = None
        down = None
        up = None
        right = None

    def MoveLeft(self):
        # returns the resulting board state after moving the blank node left        

        #return child
        pass

    def MoveDown(self):
        # returns the resulting board state after moving the blank node down

        #return child
        pass

    def MoveUp(self):
        # returns the resulting board state after moving the blank node up

        #return child
        pass

    def MoveRight(self):
        # returns the resulting board state after moving the blank node right
       
        #return child
        pass

    def UniformCost(self):
        # uniform cost search
        #GoalState = ...
        pass