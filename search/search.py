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

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
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
    
    
    '''
    def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

    def a_star_search(graph, start, goal):
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0
        
        while not frontier.empty():
            current = frontier.get()
            
            if current == goal:
                break
            
            for next in graph.neighbors(current):
                new_cost = cost_so_far[current] + graph.cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current
        
    return came_from, cost_so_far
    
    from implementation import *
    start, goal = (1, 4), (7, 8)
    came_from, cost_so_far = a_star_search(diagram4, start, goal)
    draw_grid(diagram4, width=3, point_to=came_from, start=start, goal=goal)
    print()
    draw_grid(diagram4, width=3, number=cost_so_far, start=start, goal=goal)
    print()
    
    '''
    
    '''
    
    def aStarSearch(problem, heuristic=nullHeuristic):
        "Search the node that has the lowest combined cost and heuristic first."
        "*** YOUR CODE HERE ***"
        frontier = util.PriorityQueue()
        visited = dict()

        state = problem.getStartState()
        node = {}
        node["parent"] = None
        node["action"] = None
        node["state"] = state
        node["cost"] = 0
        node["eval"] = heuristic(state, problem)
        # A* use f(n) = g(n) + h(n)
        frontier.push(node, node["cost"] + node["eval"])

        while not frontier.isEmpty():
            node = frontier.pop()
            state = node["state"]
            cost = node["cost"]
            v = node["eval"]

            if visited.has_key(state):
            continue

            visited[state] = True
            if problem.isGoalState(state) == True:
            break

            for child in problem.getSuccessors(state):
            if not visited.has_key(child[0]):
                sub_node = {}
                sub_node["parent"] = node
                sub_node["state"] = child[0]
                sub_node["action"] = child[1]
                sub_node["cost"] = child[2] + cost
                sub_node["eval"] = heuristic(sub_node["state"], problem)
                frontier.push(sub_node, sub_node["cost"] + node["eval"])

        actions = []
        while node["action"] != None:
            actions.insert(0, node["action"])
            node = node["parent"]

        return actions
            '''
            util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
