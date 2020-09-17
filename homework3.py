"""
CS561 HW1
Searching Algorithms
BFS, UCS, A*
"""
import sys
import os
import math
import heapq
import time
from collections import deque
#18 actions 
dx = [1, -1, 0, 0, 0, 0, 1, 1, -1, -1, 1, 1, -1, -1, 0, 0, 0, 0]
dy = [0, 0, 1, -1, 0, 0, 1, -1, 1, -1, 0, 0, 0, 0, 1, 1, -1, -1]
dz = [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1]


def FindPath(exit_point, short_path, entry_point):
    target = ' '.join([str(i) for i in exit_point])+ ' ' +str(short_path[tuple(exit_point)][2])
    path = [target]
    pre = exit_point
    while pre != entry_point:
        pre = short_path[tuple(pre)][1]
        tmp = ' '.join([str(i) for i in pre])+' '+str(short_path[tuple(pre)][2])
        path.append(tmp)
    path.reverse()
    return path

def GetOutput(curmin, finalsteps, exit_point, short_path, entry_point):
    if curmin == sys.maxsize:
        f1 = open("output.txt", "w")
        f1.write("FAIL")
        f1.write("\n")
        f1.close()
    else:
        path = FindPath(exit_point, short_path, entry_point)
        f1 = open("output.txt", "w")   
        print("open a file")          
        f1.write(str(curmin))
        f1.write('\n')
        f1.write(str(finalsteps))
        f1.write('\n')
        for j in range(len(path)):
            s = path[j]
            f1.write(s)
            f1.write('\n')
        f1.close()

def BFS(entry_point, exit_point, n, actions, maze_size):
    print("The Function BFS begins")
    curmin = sys.maxsize
    finalsteps = sys.maxsize
    #store the path to arrive at each point
    short_path = {}
    short_path[tuple(entry_point)] = (0, entry_point, 0)
    #create a queue for BFS search
    queue = deque()
    queue.append((0, entry_point, 1))
    #record the points that has been visited already
    visited = set()

    while queue:
        tmp = queue.popleft()

        cost = tmp[0]
        cur_point = tmp[1]
        steps = tmp[2]
        if tuple(cur_point) in visited:
            continue
        #store the visited grid
        visited.add(tuple(cur_point))

        #if arrived at the exit, compare the cost with the previous path
        if cur_point == exit_point:
            #update the current min cost
            curmin = cost
            finalsteps = steps 
            break     

        #find the available actions at this point
        if tuple(cur_point) not in actions.keys():
            continue
        else:
            move = actions[tuple(cur_point)]
        #print('next action is', move)
        cost += 1
        steps += 1
        for j in range(len(move)):
            point = cur_point[:]
            #find the next grid that the current grid can get to
            idx = move[j]-1
            point[0] += dx[idx]
            point[1] += dy[idx]
            point[2] += dz[idx]
            #make sure the grid is in the maze
            if point[0] <= maze_size[0] and point[1] <= maze_size[1] and point[2] <= maze_size[2]:
                pre_cost = short_path[tuple(cur_point)][0]
                pre_path = short_path[tuple(cur_point)][1]
                if tuple(point) in short_path.keys():
                    if cost < pre_cost:
                        short_path[tuple(point)] = (cost, cur_point, 1)
                        queue.remove((pre_cost, point, steps-1))
                else:
                    short_path[tuple(point)] = (cost, cur_point, 1)
                queue.append((cost, point, steps))
    GetOutput(curmin, finalsteps, exit_point, short_path, entry_point)

def UCS(entry_point, exit_point, n, actions, maze_size):
    print("The Function UCS begins")

    curmin = sys.maxsize
    finalsteps = sys.maxsize
    #store the path to arrive at each point
    short_path = {}
    short_path[tuple(entry_point)] = (0, entry_point, 0)
    #create a queue for UCS search
    heap = []
    heapq.heappush(heap, (0, entry_point, 1))
    #record the points that has been visited already
    visited = set()

    while heap:
        tmp = heapq.heappop(heap)

        cost = tmp[0]
        cur_point = tmp[1]
        steps = tmp[2]
        if tuple(cur_point) in visited:
            continue
        visited.add(tuple(cur_point))

        if cur_point == exit_point:
            #update the current min cost
            curmin = cost
            finalsteps = steps 

        if tuple(cur_point) not in actions.keys():
            continue
        else:
            move = actions[tuple(cur_point)]
        #print('next action is', move)
        steps += 1
        for j in range(len(move)):
            point = cur_point[:]
            path_cost = cost                            
            #find the next grid
            idx = move[j]-1
            point[0] += dx[idx]
            point[1] += dy[idx]
            point[2] += dz[idx]
            #find the move is a straight move or diagonal move
            move_cost = 10 if move[j] <= 6 else 14
            if point[0] <= maze_size[0] and point[1] <= maze_size[1] and point[2] <= maze_size[2]:
                pre_cost = short_path[tuple(cur_point)][0]
                pre_path = short_path[tuple(cur_point)][1]
                path_cost += move_cost
                if tuple(point) in short_path.keys():
                    if path_cost < pre_cost:
                        short_path[tuple(point)] = (path_cost, cur_point, move_cost)
                        heap.remove((pre_cost, point, steps-1))
                else:
                    short_path[tuple(point)] = (path_cost, cur_point, move_cost)
                heapq.heappush(heap, (path_cost, point, steps))

    GetOutput(curmin, finalsteps, exit_point, short_path, entry_point)

def A_star(entry_point, exit_point, n, actions, maze_size):
    print("The Function A_star begins")

    curmin = sys.maxsize
    finalsteps = sys.maxsize
    #store the path to arrive at each point
    short_path = {}
    short_path[tuple(entry_point)] = (0, entry_point, 0)
    #create a queue for A* search
    heap = []
    heapq.heappush(heap, (0, 0, entry_point, 1))
    #record the points that has been visited already
    visited = set()

    #function to get h(n)
    def Heuristics(exit_point, grid):
        return abs(grid[0]-exit_point[0])+abs(grid[1]-exit_point[1])+abs(grid[1]-exit_point[1])

    while heap:
        tmp = heapq.heappop(heap)

        f = tmp[0]
        cost = tmp[1]
        cur_point = tmp[2]
        steps = tmp[3]
        if tuple(cur_point) in visited:
            continue
        visited.add(tuple(cur_point))

        if cur_point == exit_point:
            #update the current min cost
            curmin = cost
            finalsteps = steps

        #find the available actions at this point
        if tuple(cur_point) not in actions.keys():
            continue
        else:
            move = actions[tuple(cur_point)]
        #print('next action is', move)
        steps += 1
        for j in range(len(move)):
            point = cur_point[:]
            path_cost = cost
            move_cost = cost
            cur_f = f
            #find the next grid
            idx = move[j]-1
            point[0] += dx[idx]
            point[1] += dy[idx]
            point[2] += dz[idx]
            #find the move is a straight move or diagonal move
            move_cost = 10 if move[j] <= 6 else 14
            path_cost += move_cost
            cur_f = path_cost+Heuristics(exit_point, point)
            if point[0] <= maze_size[0] and point[1] <= maze_size[1] and point[2] <= maze_size[2]:
                pre_cost = short_path[tuple(cur_point)][0]
                pre_path = short_path[tuple(cur_point)][1]
                if tuple(point) in short_path.keys():
                    if path_cost < pre_cost:
                        short_path[tuple(point)] = (cost, cur_point, move_cost)
                        heap.remove((pre_cost, point, steps-1))
                else:
                    short_path[tuple(point)] = (cost, cur_point, move_cost)
                heapq.heappush(heap, (cur_f, path_cost, point, steps))
    
    GetOutput(curmin, finalsteps, exit_point, short_path, entry_point)

if __name__ == "__main__":
    #begin the clock to calculate the time
    t1 = time.time()
    #read from the input file to get the variables
    f = open("input.txt")
    method = f.readline().strip()
    line = f.readline().strip()
    maze_size = line.split(' ')
    maze_size = list(map(int, maze_size))

    line = f.readline().strip()
    entry_point= line.split(' ')
    entry_point = list(map(int, entry_point))

    line = f.readline().strip()
    exit_point = line.split(' ')
    exit_point = list(map(int,exit_point))

    line = f.readline().strip()
    n= int(line)

    actions = {}
    line = f.readline().strip()
    while line:
        tmp = line.split(' ')
        tmp = list(map(int, tmp))
        actions[tuple(tmp[0:3])] = tmp[3:]
        line = f.readline().strip()
    f.close()

    if method == "BFS":
        #print("The method is BFS")
        BFS(entry_point, exit_point, n, actions, maze_size)
    elif method == "UCS":
        #print("The method is UCS")
        UCS(entry_point, exit_point, n, actions, maze_size)
    else:
        #print("The method is A*")
        A_star(entry_point, exit_point, n, actions, maze_size)

    print('total time cost is {}'.format(time.time()-t1))