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
        pass
    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        pass

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        pass
    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        pass


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def make_actions_array(node, parent):
    return [] if node[1] is None else make_actions_array(parent[node], parent) + [node[1]]

def costless_search(problem: SearchProblem, struct: util.Stack or util.Queue):
    par = dict()
    reached = set()
    struct.push((problem.getStartState(), None))

    while not struct.isEmpty():
        node = struct.pop()
        curState = node[0]
        if problem.isGoalState(curState):
            return make_actions_array(node, par)
        if curState in reached:
            continue
        reached.add(curState)
        for next in problem.getSuccessors(curState):
            if next[0] not in reached:
                struct.push(next)
                par[next] = node
    return []

def costful_search(problem: SearchProblem, heuristic):
    parent = dict()
    reached = set()
    cost = dict()
    pq = util.PriorityQueue()
    start_state = problem.getStartState()
    pq.push(item=(start_state, None), priority=0)
    cost[start_state] = 0
    while not pq.isEmpty():
        node = pq.pop()
        curState = node[0]
        if problem.isGoalState(curState):
            return make_actions_array(node, parent)
        if curState in reached:
            continue
        reached.add(curState)
        for next in problem.getSuccessors(curState):
            next_state, next_cost = next[0], next[2]
            if next_state not in reached:
                g = cost[curState] + next_cost
                h = heuristic(next_state, problem)
                pq.update(next[:2], g+h)
                cost[next_state] = g
                parent[next[:2]] = node

    return []


def depthFirstSearch(problem: SearchProblem):
    return costless_search(problem, util.Stack())
        

def breadthFirstSearch(problem: SearchProblem):
    return costless_search(problem, util.Queue())

def uniformCostSearch(problem: SearchProblem):   
    return costful_search(problem=problem, heuristic=nullHeuristic)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    return costful_search(problem, heuristic)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
