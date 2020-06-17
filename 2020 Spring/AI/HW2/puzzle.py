
from __future__ import division
from __future__ import print_function
from heapq import heappush, heappop
from collections import deque


import sys
import math
import time
import queue as Q
import psutil
#### SKELETON CODE ####
## The Class that Represents the Puzzle
class PuzzleState(object):
    """
        The PuzzleState stores a board configuration and implements
        movement instructions to generate valid children.
    """
    def __init__(self, config, n, parent=None, action="Initial", cost=0):
        """
        :param config->List : Represents the n*n board, for e.g. [0,1,2,3,4,5,6,7,8] represents the goal state.
        :param n->int : Size of the board
        :param parent->PuzzleState
        :param action->string
        :param cost->int
        """
        if n*n != len(config) or n < 2:
            raise Exception("The length of config is not correct!")
        if set(config) != set(range(n*n)):
            raise Exception("Config contains invalid/duplicate entries : ", config)

        self.n        = n
        self.cost     = cost
        self.parent   = parent
        self.action   = action
        self.config   = config
        self.children = []
        # Get the index and (row, col) of empty block
        self.blank_index = self.config.index(0)
    def __lt__(self, other):
        return (self.total_cost, self.heap_order) < (other.total_cost, other.heap_order)
    
    def display(self):
        """ Display this Puzzle state as a n*n board """
        for i in range(self.n):
            print(self.config[3*i : 3*(i+1)])

    def move_up(self):
        """ 
        Moves the blank tile one row up.
        :return a PuzzleState with the new configuration
        """
        copy = self.config[:]
        arg = copy.index(0)
        if arg < 3:
            return None
        copy[arg] = copy[arg-3]
        copy[arg-3] = 0
        return PuzzleState(copy,self.n,parent = self,action="Up",cost = self.cost+1)
      
    def move_down(self):
        """
        Moves the blank tile one row down.
        :return a PuzzleState with the new configuration
        """
        copy = self.config[:]
        arg = copy.index(0)
        if arg > 5:
            return None
        copy[arg] = copy[arg+3]
        copy[arg+3] = 0
        return PuzzleState(copy,self.n,parent = self,action="Down",cost = self.cost+1)
      
    def move_left(self):
        """
        Moves the blank tile one column to the left.
        :return a PuzzleState with the new configuration
        """
        copy = self.config[:]
        arg = copy.index(0)
        if arg%3 == 0:
            return None
        copy[arg] = copy[arg-1]
        copy[arg-1] = 0
        return PuzzleState(copy,self.n,parent = self,action="Left",cost = self.cost+1)

    def move_right(self):
        """
        Moves the blank tile one column to the right.
        :return a PuzzleState with the new configuration
        """
        copy = self.config[:]
        arg = copy.index(0)
        if (arg+1)%3 == 0:
            return None
        copy[arg] = copy[arg+1]
        copy[arg+1] = 0
        return PuzzleState(copy,self.n,parent = self,action="Right",cost = self.cost+1)
      
    def expand(self):
        """ Generate the child nodes of this node """
        
        # Node has already been expanded
        if len(self.children) != 0:
            return self.children
        
        # Add child nodes in order of UDLR
        children = [
            self.move_up(),
            self.move_down(),
            self.move_left(),
            self.move_right()]

        # Compose self.children of all non-None children states
        self.children = [state for state in children if state is not None]
        return self.children

# Function that Writes to output.txt

### Students need to change the method to have the corresponding parameters
def writeOutput(path,node_num,max_depth,times,ram):
    ### Student Code Goes here
    cost = path[-1].cost
    p = ""
    for each in path:
        p = p +", " + "'"+each.action+"'"
    p = "[" + p[13:] + "]" + "\n"
    output = open("Output.txt","w+")
    output.write("path_to_goal: ")
    output.write(p)
    output.write("cost_of_path: ")
    output.write(str(cost))
    output.write("\n")
    output.write("nodes_expanded: ")
    output.write(str(node_num))
    output.write("\n")
    output.write("search_depth: ")
    output.write(str(cost))
    output.write("\n")
    output.write("max_search_depth: ")
    output.write(str(max_depth))
    output.write("\n")
    output.write("running_time: ")
    output.write(str(times))
    output.write("\n")
    output.write("max_ram_usage: ")
    output.write(str(ram))
    output.write("\n")
    output.close() 
    pass

def bfs_search(initial_state):
    """BFS search"""
    ### STUDENT CODE GOES HERE ###
    frontier = Q.Queue(maxsize=0)
    frontier.put(initial_state)
    frontier_config = set([])
    frontier_config.add(tuple(initial_state.config))
    explored = set([])
    child_expaned = 1
    max_depth = 1
    expanded = 1
    stime  = time.time()
    while not frontier.empty():
        state = frontier.get()
        explored.add(tuple(state.config))
        expanded += 1
        if test_goal(state):
            path = []
            final = state
            while state.parent != None:
                path.append(state.parent)
                state = state.parent
            path.reverse()
            path.append(final)
            node_num = len(explored)
            time_t = time.time() - stime
            ram = psutil.Process().memory_info().rss/(10**6)
            writeOutput(path,node_num-1,max_depth,time_t,ram)
            return
        
        for child in state.expand():
            if (tuple(child.config) not in frontier_config) and (tuple(child.config) not in explored):
                if child.cost > max_depth:
                    max_depth = child.cost
                frontier.put(child)
                child_expaned += 1
                frontier_config.add(tuple(child.config))
                
    return
def dfs_search(initial_state):
    """DFS search"""
    ### STUDENT CODE GOES HERE ###
    frontier = deque()
    frontier.append(initial_state)
    frontier_config = set([])
    frontier_config.add(tuple(initial_state.config))
    explored = set([])
    child_expaned = 1
    max_depth = 1
    expanded = 1
    stime  = time.time()
    while not len(frontier) == 0:
        state = frontier.pop()
        explored.add(tuple(state.config))
        expanded += 1
        if test_goal(state):
            path = []
            final = state
            while state.parent != None:
                path.append(state.parent)
                state = state.parent
            path.reverse()
            path.append(final)
            node_num = len(explored)
            time_t = time.time() - stime
            ram = psutil.Process().memory_info().rss/(10**6)
            writeOutput(path,node_num-1,max_depth,time_t,ram)
            
            return

        for child in reversed(state.expand()):
            if (tuple(child.config) not in frontier_config) and (tuple(child.config) not in explored):
                if child.cost > max_depth:
                    max_depth = child.cost
                
                frontier.append(child)
                child_expaned += 1
                frontier_config.add(tuple(child.config))
                
    return

def A_star_search(initial_state):
    """A * search"""
    ### STUDENT CODE GOES HERE ###
    ### STUDENT CODE GOES HERE ###
    frontier = []
    initial_state.total_cost = calculate_total_cost(initial_state)
    initial_state.heap_order = time.time()
    heappush(frontier,initial_state)
    udlr_dic = {"Up":1,"Down":2,"Left":3,"Right":4}
    frontier_config = set([])
    frontier_config.add(tuple(initial_state.config))
    explored = set([])
    child_expaned = 1
    max_depth = 1
    expanded = 1
    stime  = time.time()
    while not len(frontier) == 0:
        state = heappop(frontier)
        explored.add(tuple(state.config))
        expanded += 1
        if test_goal(state):
            path = []
            final = state
            while state.parent != None:
                path.append(state.parent)
                state = state.parent
            path.reverse()
            path.append(final)
            node_num = len(explored)
            time_t = time.time() - stime
            ram = psutil.Process().memory_info().rss/(10**6)
            writeOutput(path,node_num-1,max_depth,time_t,ram)
            return
        
        for child in state.expand():
            if (tuple(child.config) not in frontier_config) and (tuple(child.config) not in explored):
                if child.cost > max_depth:
                    max_depth = child.cost
                child.total_cost = calculate_total_cost(child)
                child.heap_order = time.time()
                heappush(frontier,child)
                child_expaned += 1
                frontier_config.add(tuple(child.config))
                
    return

def calculate_total_cost(state):
    """calculate the total estimated cost of a state"""
    ### STUDENT CODE GOES HERE ###
    
    return calculate_manhattan_dist(state.config) + state.cost;

def calculate_manhattan_dist(config):
    """calculate the manhattan distance of a tile"""
    ### STUDENT CODE GOES HERE ###
    goal = [0,1,2,3,4,5,6,7,8]
    scores = 0
    for each in config:
        if not each == 0:  
            i = config.index(each)
            j = goal.index(each)
            scores = scores + abs(int(i/3)-int(j/3)) + abs(i%3 - j%3)
    return scores

def test_goal(puzzle_state):
    """test the state is the goal state or not"""
    ### STUDENT CODE GOES HERE ###
    result = False
    if puzzle_state.config == [0,1,2,3,4,5,6,7,8]:
       result = True 
    return result

# Main Function that reads in Input and Runs corresponding Algorithm
def main():
    search_mode = sys.argv[1].lower()
    begin_state = sys.argv[2].split(",")
    begin_state = list(map(int, begin_state))
    board_size  = int(math.sqrt(len(begin_state)))
    hard_state  = PuzzleState(begin_state, board_size)
    start_time  = time.time()
    
    if   search_mode == "bfs": bfs_search(hard_state)
    elif search_mode == "dfs": dfs_search(hard_state)
    elif search_mode == "ast": A_star_search(hard_state)
    else: 
        print("Enter valid command arguments !")
        
    end_time = time.time()
    print("Program completed in %.3f second(s)"%(end_time-start_time))

if __name__ == '__main__':
    main()