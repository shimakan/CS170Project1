# CS171Project1

This project is the first project in Dr. Eamonn Keogh's Introduction to AI course at the University of California, Riverside for the Fall 2022 quarter. It 
tests different search algorithms and heuristics on the sliding tile puzzle, a puzzle whose goal it is to get a semi-randomly distributed grid of numbers into 
certain goal position, as seen in figure 1 below. The search algorithms/heuristics included are: uniform cost search, A* with misplaced tile heuristic, and A* 
with Manhattan heuristic. Any unoriginal code is from the numpy library and solely consists of routines to help make matrix manipulation easier. In the 
completion in this project I consulted:

- ”https://numpy.org/doc/stable/reference/” since I represented states as numpy arrays
- Blind and Heuristic search lecture slides
- Chapter 3 of the textbook ”Artificial Intelligence a Modern Approach” by Peter Norvig and Stuart J.
Russell

<p align = "center"><img src = "https://user-images.githubusercontent.com/72994273/201992082-60e183e7-4d14-47fd-bdde-d51912384e6b.png"></p><p align = "center">
Figure 1: 8 Puzzle
</p>

## Search Algorithms

**Uniform Cost Search**

With regard to the general search algorithm provided in the slides, uniform cost search queues nodes in order
of cheapest g(n) as seen in figure 1.1. g(n) is the cost function and returns the sum of costs from some
node back up to the initial node. Notably, uniform cost search is a special case of A* search in which the heuristic cost is hard coded to equal 0. This lack of 
heuristic makes uniform cost search a blind search algorithm, but it is still optimal and complete.

<p align = "center"><img src = "https://user-images.githubusercontent.com/72994273/201995126-5cec2249-2115-49bb-afa0-ed9e9563c2c8.png"></p><p align = "center">
Figure 1.1: Node expansion based on g(n)
</p>

**A-Star With Misplaced Tile Heuristic**

The misplaced tile is a heuristic that compares a state with the goal state and returns the sum of tiles that are
placed in the wrong position with respect to the goal state as seen in figure 1.2. A heuristic like misplaced
tile provides information about the distance to a goal state, which a search algorithm like uniform cost search
does not have. For this reason, search algorithms which employ heuristics are known as informed search
algorithms.

In figure 1.2 you can see that 8 is the only number that is out of place, not counting 0. That means
that the heuristic cost h is equal to 1.

<p align = "center"><img src = "https://user-images.githubusercontent.com/72994273/201996230-0d26268c-1fcd-413b-9210-cfaf6a348dc8.png"></p><p align = "center">
Figure 1.2: Misplaced tile heuristic with one tile out of place
</p>

**A-Star With Manhattan Heuristic**

The Manhattan heuristic is similar to the misplaced heuristic in that it compares some state to the goal
state and makes an estimate of the distance to the goal state. With regard to the sliding tile puzzle, the
Manhattan distance is a distance metric used to measure how far a tile is from its desired position in the
goal state. The Manhattan distance formula is as given: $D_{Manhattan} = |x_{2} − x_{1}| + |y_{2} − y_{1}|$.

In figure 1.2. you can see that 8 is the only tile out of place, not counting zero, and that it is one tile
slide away from it’s goal position, which makes the heuristic cost h equal to 1.

## Example based algorithm comparison

**Nodes expanded vs Solution depth**

I used the following test cases seen in figure 2.1 to test and compare each algorithm.

<p align = "center"><img src = "https://user-images.githubusercontent.com/72994273/201997711-23bcb999-1897-4cba-a68a-e5f56cb199c5.png"></p><p align = "center">
Figure 2.1: Test cases
</p>

It can be seen in figure 2.2 that uniform cost search performs much worse than A* with the misplaced
tile and Manhattan heuristic. This is expected because uniform cost search is a blind search algorithm,
meaning that it does not have information to inform it’s search. It can also be seen that the misplaced tile
heuristic performs similarly to the Manhattan search in the beginning, but the Manhattan heuristic ends up
expanding fewer nodes, making it a better heuristic than misplaced tile.

<p align = "center"><img src = "https://user-images.githubusercontent.com/72994273/201998136-990ab151-8f37-4bd4-a26f-08771475d6ea.png"></p><p align = "center">
Figure 2.2: Nodes expanded vs Solution depth
</p>
