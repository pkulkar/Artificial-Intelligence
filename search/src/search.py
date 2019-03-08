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
    StartState=problem.getStartState()
    #print"StartState:",StartState
    Start=[[StartState]]
    #print "Start", Start
    Fringe=util.Stack()
    Fringe.push(Start)
    VisitedSet=[]
    CoordinateSet=[]
    j=0
    while not Fringe.isEmpty():
        #print "-----------State"+str(j)+"----------------"
        WorkList=Fringe.pop()
        #print "Stack pop:",WorkList
        length=len(WorkList)-1
        #print "Length by stack",length
        WorkState = WorkList[length]
        #print "WorkState:",WorkState
        if WorkState[0] not in CoordinateSet:
            #print "True that the Workstate:",WorkState[0],"is not in Cordinate set which is:",CoordinateSet
            CoordinateSet.append(WorkState[0])
            VisitedSet.append(WorkState)
            #print "Visited State:",VisitedSet
            #print "Co-ordinateSet:",CoordinateSet
            Successors=problem.getSuccessors(WorkState[0])
            #print "Successors:",Successors
            SuccessorsToVisit=[]
            for i in range(len(Successors)):
                if Successors[i][0] not in CoordinateSet:
                    SuccessorsToVisit.append(Successors[i])
                #print "Successors to Visit", SuccessorsToVisit
            if len(SuccessorsToVisit)>0:
                GoalStateCheck=SuccessorsToVisit[len(SuccessorsToVisit)-1]
                #print GoalStateCheck
            if problem.isGoalState(GoalStateCheck[0]):
                Direction=[]
                for a in range(len(WorkList)):
                    if a>0:
                        Direction.append(WorkList[a][1])
                Direction.append(GoalStateCheck[1])
                return Direction

            for successor in SuccessorsToVisit:
                StateToPush = [] + WorkList
                StateToPush.append(successor)
                #print "StateToPush:",StateToPush
                Fringe.push(StateToPush)
        j+=1

    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    StartState=problem.getStartState()
    #print "StartState: ",StartState
    Queue=util.Queue()
    Queue.push((StartState,[],0))
    (State, Direction, Cost) = Queue.pop()
    #print "State: ",State
    VisitedSet=[State]
    #print "Visited Set:",VisitedSet
    j=0
    while not problem.isGoalState(State):
        #print "--------State: ",j,"-------"
        Successors=problem.getSuccessors(State)
        #print "Successors: ",Successors
        for successor in Successors:
            if problem.isGoalState(successor[0]) or successor[0] not in VisitedSet:
                #print "True that",successor[0],"is not in Visited State which is :",VisitedSet
                StateToVisit=successor[0]
                #print "StateToVisit:",StateToVisit
                newDirection=Direction+[successor[1]]
                #print "New Direction:",newDirection
                cost=1
                StateToPush=(StateToVisit,newDirection,cost)
                #print "StateToPush:",StateToPush
                VisitedSet.append(successor[0])
                #print "VisitedSetAfterAppend:",VisitedSet
                Queue.push(StateToPush)
        (State,Direction,Cost)=Queue.pop()
        #print"After Pop State:",State
        #print"After Pop Direction",Direction
        #print"After Pop Cost",Cost
        j+=1
    #print "Final Direction:",Direction
    return Direction
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    visited_nodes = []
    #print "visited_nodes",visited_nodes
    start = problem.getStartState()
    #print "start:",start
    Queue = util.PriorityQueue()
    #print "Queue:",Queue
    Queue.push((start, [], 0), 0)

    while not Queue.isEmpty():
        point, direction, cost = Queue.pop()
        #print "Point:",point
        #print "Directions:",direction
        #print  "Cost:",cost
        if point in visited_nodes:
            continue

        if problem.isGoalState(point):
            #print "Direction:",direction
            return direction

        visited_nodes.append(point)
        #print"Visited_nodes:",visited_nodes
        succ = problem.getSuccessors(point)
        for node_new, dir, new_cost in succ:
            if node_new in visited_nodes:
                continue
            newNode=node_new
            #print "newNode:",newNode
            newDirection=direction + [dir]
            newCost=cost + new_cost
            #print "newCost",newCost
            Priority=cost + new_cost
            #print "Priority:",Priority
            Queue.push((newNode, newDirection, newCost), Priority)
    return []


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    visited_node = []
    #print ("VisitedNode:",visited_node)
    Queue = util.PriorityQueue()
    #print("Queue:",Queue)
    start = problem.getStartState()
    #print("StartState:",start)
    Queue.push((start, [], 0), 0)
    #print("Pop result",Queue.pop())
    while not Queue.isEmpty():
        point, direction, cost = Queue.pop()
        if point in visited_node:
           continue

        if problem.isGoalState(point):
            #print "End Route:",direction
            return direction

        visited_node.append(point)
        succ = problem.getSuccessors(point)
        for node_new, dir, new_cost in succ:
            if node_new in visited_node:
                continue
            newNode = node_new
            newDirection = direction + [dir]
            newCost = cost + new_cost
            Priority = newCost+ heuristic(newNode, problem)
            #print "Priority:",Priority
            Queue.push((newNode, newDirection , newCost), Priority)

    return []
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch




