# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html


"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
"""

import util
from collections import namedtuple

node = namedtuple("node", "state, parent, action, pathCost")

class SearchProblem:
  """
  This class outlines the structure of a search problem, but doesn't implement
  any of the methods (in object-oriented terminology: an abstract class).
  
  You do not need to change anything in this class, ever.
  """
  
  def getStartState(self):
     """
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take
 
     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()
           

def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]


"""def genericSearch(problem):"""

def depthFirstSearch(problem):
  DFSStack = util.Stack()             #declare stack
  DFSStack.push([(problem.getStartState(), "Start" , 0)]) #first thing on stack                                       #list of visited coordinates 
  visited = [] 
  while DFSStack.isEmpty() is False:                    
      path = DFSStack.pop()
      if path[-1][0] not in visited:        #path -1 0 is the location of the next location
          for successor in problem.getSuccessors(path[-1][0]): 
              visited.append(path[-1][0]) 
              if successor[0] not in visited:
                  next = path[:]
                  next.append(successor)
                  DFSStack.push(next)
      if problem.isGoalState(path[-1][0]):
          return [directions[1] for directions in path][1:]  
  return[]

def breadthFirstSearch(problem):
  BFSQueue = util.Queue()               #declare queue
  BFSQueue.push([(problem.getStartState(), "Start" , 0)])  #first thing on queue
  visited = []                                          #list of visited coordinates 
  while BFSQueue.isEmpty() is False:
      path = BFSQueue.pop()                     #nested lists are weird
      if path[-1][0] not in visited:          
          for successor in problem.getSuccessors(path[-1][0]):  #path -1 0 is the location of the next location
              visited.append(path[-1][0])
              if successor[0] not in visited:
                  next = path[:]
                  next.append(successor)      #add next thingy
                  BFSQueue.push(next)      
      if problem.isGoalState(path[-1][0]):        #success check
          return [directions[1] for directions in path][1:]
  return[]
      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"
  util.raiseNotDefined()

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  
  AStarQueue = util.PriorityQueue()               #declare queue
  AStarH = 0
  AStarG = 0
  AStarF = AStarG + AStarH                          #formula For A*
  AStarQueue.push([(problem.getStartState(), "Start" , 0)], AStarF)  #first thing on queue
  visited = []                                          #list of visited coordinates 
  while AStarQueue.isEmpty() is False:
      path = AStarQueue.pop()
      if path[-1][0] not in visited:          
          for successor in problem.getSuccessors(path[-1][0]):
              visited.append(path[-1][0])
              if successor[0] not in visited:
                  AStarG = path[-1][2] + AStarG         #getting new g Value
                  AStarH = heuristic(path[-1][0], problem) + AStarH    #getting new H Value
                  AStarF = AStarG + AStarH              #getting F Value
                  next = path[:]
                  next.append(successor)
                  AStarQueue.push(next, AStarF)      
      if problem.isGoalState(path[-1][0]):
          return [directions[1] for directions in path][1:]
  return[]
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
