# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
from util import Stack
from util import Queue
from util import PriorityQueueWithFunction


class Node:  # No need to declare field variables outside constructor like in java
    def __init__(self, _current=None, _parent=None, _lastAction=None, _previousCost = 0):
        self.current = _current  # state
        self.parent = _parent  # state
        self.lastAction = _lastAction  # action took (direction)
        self.previousCost = _previousCost
class SearchProblem:

    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()

def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem): # return -> list of actions to take from the start point
    """
    Search the deepest nodes in the search tree first.

    Unlike the ones you see in class, your search algorithm needs to return
    a list of actions that reaches the goal (How to retrieve a list of actions
    from the goal node?). Make sure to implement a graph
    search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))

    HINT: Start by defining a Node class
    """
    start = problem.getStartState() # state: position, cornersVisited, prevPositions
    nodeStack = Stack()
    currentNode = Node(_current= start)
    nodeStack.push(currentNode)
    visited = set()

    while not nodeStack.isEmpty():
        currentNode = nodeStack.pop()
        currentState = currentNode.current

        if problem.isGoalState(currentState):
            actions = []
            while currentNode.parent is not None:  # we go back up to the initial state
                actions.append(currentNode.lastAction)
                currentNode = currentNode.parent
            actions.reverse()# to get from the start
            return actions

        if currentState not in visited:
            visited.add(currentState)
            successors = problem.getSuccessors(currentState) # Tuple: (nextState, action, cost)
            for successor in successors:#successor: state, action, cost
                newNode = Node(_current= successor[0], _parent= currentNode, _lastAction= successor[1])
                nodeStack.push(newNode)

    print("Could not find the goal state")
    return None

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    start = problem.getStartState()
    nodeQueue = Queue()
    currentNode = Node(_current = start)
    nodeQueue.push(currentNode)
    visited = set()

    while not nodeQueue.isEmpty():
        currentNode = nodeQueue.pop()
        currentState = currentNode.current

        if problem.isGoalState(currentState):
            actions = []
            while currentNode.parent is not None:  # we go back up to the initial state
                actions.append(currentNode.lastAction)
                currentNode = currentNode.parent
            actions.reverse()  # to get from the start
            return actions
        if currentState not in visited:
            visited.add(currentState)
            successors = problem.getSuccessors(currentState)  # Tuple: (nextState, action, cost)
            for successor in successors:  # successor: state, action, cost
                newNode = Node(_current=successor[0], _parent=currentNode, _lastAction=successor[1])
                nodeQueue.push(newNode)

    print("Could not find the goal state")
    return None

def uniformCostSearch(problem):
    def giveCost(node):
        actions = []
        current = node
        while current.parent is not None:
            actions.append(current.lastAction)
            current = current.parent
        actions.reverse()
        return problem.getCostOfActions(actions)
    """Search the node of least total cost first."""
    start = problem.getStartState()
    # heap = PriorityQueueWithFunction(lambda node: node.previousCost)
    heap = PriorityQueueWithFunction(giveCost)
    currentNode = Node(_current = start)
    heap.push(currentNode)
    visited = set()

    while not heap.isEmpty():
        currentNode = heap.pop()
        currentState = currentNode.current

        if problem.isGoalState(currentState):
            actions = []
            while currentNode.parent is not None:  # we go back up to the initial state
                actions.append(currentNode.lastAction)
                currentNode = currentNode.parent
            actions.reverse()  # to get from the start
            return actions

        if currentState not in visited:
            visited.add(currentState)
            successors = problem.getSuccessors(currentState)  # Tuple: (nextState, action, cost)
            for successor in successors:  # successor: state, action, cost
                newNode = Node(_current=successor[0], _parent=currentNode, _lastAction=successor[1], _previousCost= currentNode.previousCost + successor[2])
                heap.push(newNode)

    print("Could not find the goal state")
    return None

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
