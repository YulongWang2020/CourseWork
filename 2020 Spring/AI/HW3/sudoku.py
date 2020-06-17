#!/usr/bin/env python
#coding:utf-8

"""
Each sudoku board is represented as a dictionary with string keys and
int values.
e.g. my_board['A1'] = 8
"""
import numpy as np
import time
import sys
import statistics
ROW = "ABCDEFGHI"
COL = "123456789"

class csp(object):
    def __init__(self,board):
        self.variable = []
        for each in board.keys():
            if board[each]== 0:
                self.variable.append(each)
        temp = {}
        self.domain = temp.fromkeys(self.variable, [1,2,3,4,5,6,7,8,9])
        new_domain = temp.fromkeys(self.variable, [1,2,3,4,5,6,7,8,9])
        for i in range(len(self.domain.keys())):
            temp_key = list(self.domain.keys())[i]
            for j in range(len(self.domain[temp_key])):
                if not check_consistent(board,keys = temp_key,values = self.domain[temp_key][j] ,if_total = False):
                    temp_list = new_domain[temp_key].copy()
                    temp_list.remove(self.domain[temp_key][j])
                    new_domain[temp_key] = temp_list
        self.domain = new_domain
    def resolve_conflict(self,board):
        removed = {}
        temp = {}
        new_domain = temp.fromkeys(self.variable)
        for each in new_domain:
            new_domain[each] = self.domain[each]
        for i in range(len(self.domain.keys())):
            temp_key = list(self.domain.keys())[i]
            for j in range(len(self.domain[temp_key])):
                temp_value = self.domain[temp_key][j]
                if not check_consistent(board,keys = temp_key,values = self.domain[temp_key][j] ,if_total = False):
                    if temp_key in removed:
                        removed[temp_key] = removed[temp_key].append(temp_value)
                    removed[temp_key] = temp_value
                    temp_list = new_domain[temp_key].copy()
                    temp_list = temp_list[:temp_list.index(temp_value)]+temp_list[temp_list.index(temp_value)+1:]
                    new_domain[temp_key] = temp_list
        self.domain = new_domain
        return removed
    def assign_board(self,board,key,value):
        board[key] = value
        self.variable.remove(key)
        del self.domain[key]
        removed = self.resolve_conflict(board)
        return board,removed
    def remove_and_readd_domain(self,board,key,domains,removed):
        board[key] = 0
        self.variable.append(key)
        self.domain[key] = domains
        for each in removed:
            temp = self.domain[each]
            temp.append(removed[each])
            self.domain[each] = temp
        return board

def check_finished(board):
    if check_consistent(board,if_total = True):
        for each in board.values():
            if each == 0:
                return False
        return True
    return False
def check_consistent(board,keys = "A1",values = 9 ,if_total = False):
    start = time.time()
    array = np.array(list(board.values())).reshape((9,9)).astype(int)
    if if_total:
        # print_board(board)
        # col = int(converter[keys[0]])-1
        # row = int(keys[1])-1
        # print("col:",col,"row:",row)
        # print(array[:,col].tolist()[:])
        # print(array[row,:].tolist()[:])
        # temp = set(array[:,col].tolist()[:])
        # temp.discard(0)
        # print(np.count_nonzero(array[:,col]),len(temp),9 - np.count_nonzero(array[:,col]) + len(temp))
        for i in range(9):
            temp = set(array[:,i].tolist()[:])
            temp.discard(0)
            if (9 - np.count_nonzero(array[:,i]) + len(temp)) != 9:
                return False;
        for j in range(9):
            temp = set(array[j,:].tolist()[:])
            temp.discard(0)
            if (9 - np.count_nonzero(array[j,:]) + len(temp)) != 9:

                return False;
        for i in range(0,9,3):
            for j in range(0,9,3):
                temp = set(array[i:i+3,j:j+3].flatten().tolist()[:])
                temp.discard(0)
                if (9 - np.count_nonzero(array[i:i+3,j:j+3]) + len(temp)) != 9:
                    return False;
        return True


    else:
        # print_board(board)
        row = int(converter[keys[0]])-1
        col = int(keys[1])-1
        array[row,col] = values
        # print("col:",col,"row:",row)
        # print(array[:,col].tolist()[:])
        # print(array[row,:].tolist()[:])
        temp = set(array[:,col].tolist()[:])

        temp.discard(0)
        if (9 - np.count_nonzero(array[:,col]) + len(temp)) != 9:
            return False;
        temp = set(array[row,:].tolist()[:])
        temp.discard(0)
        if (9 - np.count_nonzero(array[row,:]) + len(temp)) != 9:
            return False;
        i = int(row/3)*3
        j = int(col/3)*3
        temp = set(array[i:i+3,j:j+3].flatten().tolist()[:])
        temp.discard(0)
        if (9 - np.count_nonzero(array[i:i+3,j:j+3]) + len(temp)) != 9:
            return False;
        return True

def print_board(board):
    """Helper function to print board in a square."""
    print("-----------------")
    for i in ROW:
        row = ''
        for j in COL:
            row += (str(board[i + j]) + " ")
        print(row)


def board_to_string(board):
    """Helper function to convert board dictionary to string for writing."""
    ordered_vals = []
    for r in ROW:
        for c in COL:
            ordered_vals.append(str(board[r + c]))
    return ''.join(ordered_vals)

def select_mrv(csp):
    min_choice = 9
    min_key = 0
    for each in csp.domain:
        if len(csp.domain[each]) < min_choice:
            min_choice = len(csp.domain[each])
            min_key = each
    return min_key

def backtracking(board,csp):
    """Takes a board and returns solved board."""
    # TODO: implement this
    # Generate the dictionary
    if check_finished(board):
        return board;
    problem = csp
    var = select_mrv(problem)
    temp_domain = problem.domain[var]
    for each in problem.domain[var]:
        if check_consistent(board,keys = var,values = each ,if_total = False):
            board,removed = problem.assign_board(board,var,each)
            result = backtracking(board,problem)
            check = True
            for each in csp.domain:
                if csp.domain[each] == []:
                    check = False
            if (result != False) & (check != False):
                return result
        board = problem.remove_and_readd_domain(board,var,temp_domain,removed)
    return False


if __name__ == '__main__':
    if len(sys.argv) == 1:

        #  Read boards from source.
        src_filename = 'sudokus_start.txt'
        try:
            srcfile = open(src_filename, "r")
            sudoku_list = srcfile.read()
        except:
            print("Error reading the sudoku file %s" % src_filename)
            exit()

        # Setup output file
        out_filename = 'output.txt'
        outfile = open(out_filename, "w")
        # MODIFIED BY YULONG WANG
        converter = {ROW[i]: COL[i] for i in range(9)} 
        ##############
        time_list = []
        # Solve each board using backtracking
        for line in sudoku_list.split("\n"):

            if len(line) < 9:
                continue
            # Parse boards to dict representation, scanning board L to R, Up to Down
            board = { ROW[r] + COL[c]: int(line[9*r+c])
                      for r in range(9) for c in range(9)}

            # Print starting board. TODO: Comment this out when timing runs.
            print_board(board)
            problem = csp(board)
            # Solve with backtracking
            start_time = time.time()
            solved_board = backtracking(board,problem)
            # print(solved_board.values())
            total_time = time.time()-start_time
            time_list.append(total_time)
            print_board(solved_board)
            # Print solved board. TODO: Comment this out when timing runs.

            # Write board to file
            outfile.write(board_to_string(solved_board))
            outfile.write('\n')
        print("Finishing all boards in file.")
        min_time = min(time_list)
        max_time = max(time_list)
        mean_time = statistics.mean(time_list)
        sdv = statistics.stdev(time_list)
        outfile.close()
        readme = 'README.txt'
        readmefile = open(readme, "w")
        readmefile.write("Min_time: " + str(min_time))
        readmefile.write('\n')
        readmefile.write("Max_time: " + str(max_time))
        readmefile.write('\n')
        readmefile.write("Mean_time: " + str(mean_time))
        readmefile.write('\n')
        readmefile.write("Standard Deviation: " + str(sdv))
        readmefile.close()

    else: 
        out_filename = 'output.txt'
        outfile = open(out_filename, "w")
        # MODIFIED BY YULONG WANG
        converter = {ROW[i]: COL[i] for i in range(9)} 
        line = sys.argv[1]
        board = { ROW[r] + COL[c]: int(line[9*r+c])
                      for r in range(9) for c in range(9)}
        print_board(board)
        problem = csp(board)
        # Solve with backtracking
        solved_board = backtracking(board,problem)
        # print(solved_board.values())
        print_board(solved_board)
        # Print solved board. TODO: Comment this out when timing runs.

        # Write board to file
        outfile.write(board_to_string(solved_board))
        outfile.write('\n')
        outfile.close()