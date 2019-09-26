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


    fronteira = util.PriorityQueue()
    visitado = dict()
    estado = problem.getStartState()



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

    # Inicializando variaveis
    borda = util.PriorityQueue()
    visitado = []

    # Colocando o primeiro no na fornteira
    # Adicionando (estado e acao) + prioridade heuristica
    borda.push( (problem.getStartState(), []), heuristic(problem.getStartState(), problem) )
    visitado.append(problem.getStartState())


    # Note:
    # Como borda e uma priorityqueu
    # Quando coloco .push
    # A prioridade fica custo + heuristica
    # Assim, quando dou pop sairia o elemento com menor
    # valor de euristica + custo

    # Gera o loop
    while not borda.isEmpty():
        # Adiciona em no o elemento da fronteira
        # com menor custo + heuristica
        estado, acao = borda.pop()

        # Estados de Terminos:
        #####################################################
        # Se cheguei no estado, termino o loop
        if problem.isGoalState(estado) == True:
            return acao
        #####################################################

        # Se o estado nao foi visitado, adicione o estado
        if estado not in visitado:
            visitado.append(estado)

        # Loop
        # filho: (estado, acao, numero)
        for filho in problem.getSuccessors(estado):


            # Se estado nao visitado adicione na fronteira
            if filho[0] not in visitado:
                # Adicionando na borda o no caso nao esteja na borda
                # e caso haja, atualiza a lista de prioridade
                # Isso permite nao se preocupar quando adicionar
                # mesmo no , mas com prioridade menor
                borda.update((filho[0], acao + [filho[1]]), \
                   problem.getCostOfActions(acao+[filho[1]])+heuristic(filho[0], problem) )




# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
