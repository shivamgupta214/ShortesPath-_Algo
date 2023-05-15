import pandas as pd
from queue import PriorityQueue
import sys
import time


class Node:                                     #took this class code which is provided by the professor in discussion                                                   
    def __init__(self, state, parent, pathCost, heuristics, algorithm):
        self.STATE = state                  
        self.PARENT = parent                
        self.PATHCOST = pathCost            
        self.HEURISTICS = heuristics        
        if algorithm == 'GBFS':
            self.EVAL = self.HEURISTICS
        elif algorithm == 'ASTAR':
            self.EVAL = self.PATHCOST + self.HEURISTICS
        
    def getState(self):
        return self.STATE
        
    def getParent(self):
        return self.PARENT

    def getPathCost(self):
        return self.PATHCOST
        
    def getHeuristics(self):
        return self.HEURISTICS
      
    def getEval(self):
        return self.EVAL
      
    def __lt__(self, other):
        return self.getEval() < other.getEval()

   

def doSearch(INITIAL, GOAL, s_df, d_df, algorithm):                #return solution                                      
    num_expNodes = 0                                                          #initialize a variable to get number of expanded nodes
    initial_heuristics = findHeuristics(INITIAL, GOAL, s_df)       #initially we find straight line distance of initial state from goal state
    node = Node(state=INITIAL, parent= None, pathCost=0, heuristics=initial_heuristics, algorithm=algorithm)
    frontier = PriorityQueue()
    frontier.put(node)      #put initial state in queue
    reached = {}            #lookup table 
    while not frontier.empty():
        node = frontier.get() 
        if node.getState() == GOAL:  #if INITIAL and GOAL is same
            return node, num_expNodes
        else:
            node_list = ExpandNode(node, GOAL, s_df, d_df, algorithm)
            num_expNodes = num_expNodes + len(node_list)   #call ExpandNode function to find connected nodes
            for child in node_list:
                s = child.getState()
                if s not in reached or child.getEval() < reached[s].getEval():  #if reached through a path with lower PATH-COST than previously.
                    reached[node.getState()] = node
                    frontier.put(child)
    
    return None


def ExpandNode(node, GOAL, s_df, d_df, algorithm):                          #Find connected nodes
    expandNode_List = []
    s = node.getState()
    for i in range(len(s_df[s])):
        if d_df.iloc[i][s] > 0:                                             #check for connecting states with help of driving data, in which the states which are connected by edge have value greater than 0
            s2 = d_df['STATE'].iloc[i]                                      
            new_heuristics = findHeuristics(s2, GOAL, s_df)                 #find straight line distance of next connected state from goal state 
            cost = node.getPathCost() + d_df.iloc[i][s]
            expandNode_List.append(Node(state=s2, parent= node, pathCost=cost, heuristics=new_heuristics, algorithm=algorithm))                     
    return expandNode_List 
                
        
def findHeuristics(state, GOAL, s_df):                #find hueristic values for GOAL State       
        for index in range(len(s_df['STATE'])):
            if s_df['STATE'].iloc[index] == state:    #condition to find the straight line distance of state i.e., current state from GOAL state
                return s_df.iloc[index][GOAL]           
                


d_df = pd.read_csv ('driving.csv')              #driving data
s_df = pd.read_csv('straightline.csv')          #straight line distance


INITIAL = ''
GOAL = ''
inputArguments = sys.argv  #input arguments
if len(inputArguments)==3: 
    
    INITIAL = inputArguments[1]     #startState
    GOAL = inputArguments[2]        #goalState
else: # wrong input, program quits
    print("ERROR: Not enough or too many input arguments.")
    quit()

if GOAL in s_df['STATE'].values and INITIAL in s_df['STATE'].values:      #condition to check if INITIAL and GOAL state exist in data
    #start timer
    timer_GBFS =  time.time()
    result_GBFS, num_expNode = doSearch(INITIAL, GOAL, s_df, d_df, 'GBFS')
    #end timer
    timerStop_GBFS = time.time()
    executionTime_GBFS = timerStop_GBFS-timer_GBFS

    if result_GBFS == None:         #path not found
        print('\nGreedy Best First Search: ')
        print('Solution path: FAILURE: NOT FOUND' )
        print('Path cost: 0')
        print(f'Execution time: {executionTime_GBFS :.3f} seconds') 
    else:
            
        pathCost_GBFS = result_GBFS.getPathCost()
        path_GBFS=[]
        path_GBFS.append(result_GBFS.getState())
        while result_GBFS.getParent() != None:
            result_GBFS = result_GBFS.getParent()
            path_GBFS.append(result_GBFS.getState())
                     
        path_GBFS.reverse()
        print('Gupta, Shivam, A20520254 solution:')
        print('Initial State:', INITIAL)
        print('Goal State:', GOAL)

        print('\nGreedy Best First Search:')
        print(f'Solution path: {path_GBFS}')
        print(f'Number of states on a path: {len(path_GBFS)}')
        print('Number of expanded nodes:', num_expNode+1)
        print(f'Path cost: {pathCost_GBFS}')
        print(f'Execution time: {executionTime_GBFS :.3f} seconds')


    #start timer
    timer_ASTAR = time.time()
    result_ASTAR, num_expNode = doSearch(INITIAL, GOAL, s_df, d_df, 'ASTAR')
    #end timer
    timerStop_ASTAR = time.time()
    executionTime_ASTAR = timerStop_ASTAR-timer_ASTAR

    if result_ASTAR == None:               #path not found 
        print('\nA* Search: ')
        print('Solution path: FAILURE: NOT FOUND' )
        print('Path cost: 0')
        print(f'Execution time: {executionTime_ASTAR :.3f} seconds')   
    else:
        pathCost_ASTAR = result_ASTAR.getPathCost()
        path_ASTAR=[]
        path_ASTAR.append(result_ASTAR.getState())
        while result_ASTAR.getParent() != None:
            result_ASTAR = result_ASTAR.getParent()
            path_ASTAR.append(result_ASTAR.getState()) 
        path_ASTAR.reverse()
        print('\nA* Search: ')
        print(f'Solution path: {path_ASTAR}') 
        print(f'Number of states on a path: {len(path_ASTAR)}')
        print('Number of expanded nodes:', num_expNode+1)
        print(f'Path cost: {pathCost_ASTAR}')
        print(f'Execution time: {executionTime_ASTAR :.3f} seconds')
    
else:
    print('\nGreedy Best First Search: ')
    print('Solution path: FAILURE: NOT FOUND' )
    print('Path cost: 0')
    print('\nA* Search: ')
    print('Solution path: FAILURE: NOT FOUND' )
    print('Path cost: 0')