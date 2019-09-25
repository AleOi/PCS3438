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
import math as m
import pdb

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
    print("Null")
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    fronteira = util.PriorityQueue()
    visitado = dict()
    estado = problem.getStartState()

    # Colocando o primeiro no na fornteira
    no = {}
    no["pai"] = None
    no["acao"] = None
    no["estado"] = estado
    no["custo"] = 0
    no["heur"] = heuristic(estado, problem)
    fronteira.push(no, no["custo"] + no["heur"])

    # Gera o loop
    while not fronteira.isEmpty():
        # Adiciona em no o elemento da fronteira
        no = fronteira.pop()
        estado = no["estado"]
        custo = no["custo"]
        v = no["heur"]

        # Se o estado foi visitado, pule
        if visitado.has_key(estado):
            continue

        # Adiciona o estado em visitado
        # e diz que foi visitado
        visitado[estado] = True
        # Se cheguei no estado, termino o loop
        if problem.isGoalState(estado) == True:
            break
        for filho in problem.getSuccessors(estado):
            # Se filho nao visitado adicione na fronteira
            if not visitado.has_key(filho[0]):
                sub_no = {}
                sub_no["pai"] = no
                sub_no["estado"] = filho[0]
                sub_no["acao"] = filho[1]
                sub_no["custo"] = filho[2] + custo
                sub_no["heur"] = heuristic(sub_no["estado"], problem)
                # Adicionando na fronteira o no e a soma entre custo e
                pdb.set_trace()
                # heuristica
                fronteira.push(sub_no, sub_no["custo"] + sub_no["heur"])
    acao = []
    # Construindo o caminho
    # Pegue no dicionario no o outro dicionario pai
    # E vai adicionando em acao
    while no["acao"] != None:
        # Coloque no comeco da lista
        acao.insert(0, no["acao"])
        # Pegue o no pai (atualiza no)
        no = no["pai"]
    return acao

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
