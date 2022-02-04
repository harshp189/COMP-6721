2   # search.py
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

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"


    #stack for pushing the states as they are explored from the start state
    open_stack = util.Stack()

    #list for storing all the visited states
    closed_list = []

    initialStartState = problem.getStartState()

    open_stack.push((initialStartState,[]))

    while not open_stack.isEmpty():
        currentState , list_of_actions = open_stack.pop()

        if currentState not in closed_list:
            closed_list.append(currentState)

            if problem.isGoalState(currentState):
                return list_of_actions

            else:

                #retrieve the successors of the current state/node and push the new state and action onto the stack
                currentStateSuccessors = problem.getSuccessors(currentState)
                for SuccessorState,SuccessorAction,SuccessorCost in currentStateSuccessors:
                        nextAction = list_of_actions + [SuccessorAction]
                        open_stack.push((SuccessorState,nextAction))

    #will return the list of actions necessary to reach the goal state
    return list_of_actions


    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    # queue for maintaining the states as they are explored from the start state
    open_queue = util.Queue()

    # list for storing all the visited states
    closed_list = []

    initialStartState = problem.getStartState()

    open_queue.push((initialStartState, []))

    while not open_queue.isEmpty():
        currentState, list_of_actions = open_queue.pop()

        if currentState not in closed_list:
            closed_list.append(currentState)

            if problem.isGoalState(currentState):
                return list_of_actions

            else:

                # retrieve the successors of the current state/node and push the new state and action onto the stack
                currentStateSuccessors = problem.getSuccessors(currentState)
                for SuccessorState, SuccessorAction, SuccessorCost in currentStateSuccessors:
                    nextAction = list_of_actions + [SuccessorAction]
                    open_queue.push((SuccessorState, nextAction))

    # will return the list of actions necessary to reach the goal state
    return list_of_actions

    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    # priority queue for maintaining the order in which the states are returned according to lowest cost(priority)
    p_queue = util.PriorityQueue()

    # list for storing all the visited nodes
    visited_nodes = []
    priority = 0

    # the path leading to the goal state which initially is empty
    goal_path = []
    initialStartState = problem.getStartState()
    StartPoint = (initialStartState, goal_path, priority)

    # initially pushing the start point, goal path and priority to the priority queue
    p_queue.push(StartPoint, priority)

    while p_queue:
        currentState, goal_path, current_cost = p_queue.pop()

        # here we are expanding any state's successor when they are added in the visited_nodes list
        if currentState not in visited_nodes:
            visited_nodes.append(currentState)
            if problem.isGoalState(currentState):
                return goal_path
            else:
                currentStateSuccessors = problem.getSuccessors(currentState)
                for SuccessorState, SuccessorPath, SuccessorCost in currentStateSuccessors:
                    updatedPath = goal_path + [SuccessorPath]
                    updatedCost = current_cost + SuccessorCost

                    # updating the priority queue with new path ,new cost and new priority when any state is expanded and it's successors are visited
                    p_queue.update((SuccessorState, updatedPath, updatedCost), updatedCost)


    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    # priority queue for maintaining the order in which the states are returned according to lowest cost(priority)
    p_queue = util.PriorityQueue()

    # list for storing all the visited nodes
    visited_nodes = []
    priority = 0

    # the path leading to the goal state which initially is empty
    goal_path = []
    initialStartState = problem.getStartState()
    StartPoint = (initialStartState, goal_path, priority)

    # initially pushing the start point, goal path and priority to the priority queue
    p_queue.push(StartPoint, priority)

    while p_queue:
        currentState, goal_path, current_cost = p_queue.pop()

        # here we are expanding any state's successor when they are added in the visited_nodes list
        if currentState not in visited_nodes:
            visited_nodes.append(currentState)
            if problem.isGoalState(currentState):
                return goal_path
            else:
                currentStateSuccessors = problem.getSuccessors(currentState)
                for SuccessorState, SuccessorPath, SuccessorCost in currentStateSuccessors:
                    updatedPath = goal_path + [SuccessorPath]
                    updatedCost = current_cost + SuccessorCost
                    # heuristicValue
                    heuristicValue = heuristic(SuccessorState, problem)

                    # final cost = (updated cost) actual cost from the state (x) to another state (y)  + (heuristic value) estimated cost from that state (y) to the goal state
                    finalCost = updatedCost + heuristicValue

                    # updating the priority queue with new path ,new cost and new final cost (includes heuristic cost) when any state is expanded and it's successors are visited
                    p_queue.update((SuccessorState, updatedPath, updatedCost), finalCost)


    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
