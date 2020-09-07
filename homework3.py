"""
CS561 HW1
Searching Algorithms
"""
import sys
import os
import math
import heapq
from collections import deque
#18 actions 
dx = [1, -1, 0, 0, 0, 0, 1, 1, -1, -1, 1, 1, -1, -1, 0, 0, 0, 0]
dy = [0, 0, 1, -1, 0, 0, 1, -1, 1, -1, 0, 0, 0, 0, 1, 1, -1, -1]
dz = [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1]


def BFS(entry_point, exit_point, n, actions, maze_size):
    #print("The Function BFS begins")

    curmin = sys.maxsize
    #create a queue for BFS search
    queue = deque()
    entry_point.append(0)
    queue.append(([entry_point], 0, 1))
    #record the points that has been visited already
    visited = []

    #f1 = open("output.txt", "a")
    while queue:
        tmp = queue.popleft()

        cur_path = tmp[0][:]
        cur_point = cur_path[-1][0:3][:]
        cur_cost = tmp[1]
        cur_steps = tmp[2]
        #store the visited grid
        if cur_point in visited:
            continue
        visited.append(cur_point)

        #if arrived at the exit, compare the cost with the previous path
        if cur_point == exit_point:
            if cur_cost < curmin:
                #update the current min cost
                curmin = cur_cost

                f1 = open("output.txt", "w")             
                f1.write(str(cur_cost))
                f1.write('\n')
                f1.write(str(cur_steps))
                f1.write('\n')
                for j in range(len(cur_path)):
                    s = ' '.join([str(k) for k in cur_path[j]])
                    f1.write(s)
                    f1.write('\n')
                f1.close()

        i = 0
        while i < len(actions) and cur_point != actions[i][0:3]:
            i += 1
        if i < len(actions):
            move = actions[i][3:]
            #print('next action is', move)
            cur_cost += 1
            cur_steps += 1
            for j in range(len(move)):
                path = cur_path[:]
                point = cur_point[:]
                #find the next grid that the current grid can get to
                idx = move[j]-1
                point[0] += dx[idx]
                point[1] += dy[idx]
                point[2] += dz[idx]
                #make sure the grid is in the maze
                if point[0] <= maze_size[0] and point[1] <= maze_size[1] and point[2] <= maze_size[2]:
                    if point not in visited:
                        point.append(1)
                        path.append(point)
                        queue.append((path, cur_cost, cur_steps))

    if curmin == sys.maxsize:
        f1 = open("output.txt", "w")
        f1.write("FAIL")
        f1.write("\n")
    f1.close()

def UCS(entry_point, exit_point, n, actions, maze_size):
    #print("The Function UCS begins")

    curmin = sys.maxsize
    #create a queue for UCS search
    heap = []
    entry_point.append(0)
    heapq.heappush(heap, (0, [entry_point], 1))
    #record the points that has been visited already
    visited = []

    while heap:
        tmp = heapq.heappop(heap)

        cost = tmp[0]
        cur_path = tmp[1][:]
        cur_point = cur_path[-1][0:3][:]
        steps = tmp[2]
        if cur_point in visited:
            continue
        visited.append(cur_point)

        if cur_point == exit_point:
            if cost < curmin:
                #update the current min cost
                curmin = cost

                f2 = open("output.txt", "w")           
                f2.write(str(cost))
                f2.write('\n')
                f2.write(str(steps))
                f2.write('\n')
                for j in range(len(cur_path)):
                    s = ' '.join([str(k) for k in cur_path[j]])
                    f2.write(s)
                    f2.write('\n')
                f2.close()

        i = 0
        while i<len(actions) and cur_point != actions[i][0:3]:
            i+=1
        if i < len(actions):
            move = actions[i][3:]
            #print('next action is', move)
            steps += 1
            for j in range(len(move)):
                point = cur_point[:]
                path_cost = cost
                move_cost = cost
                path = cur_path[:]
                #find the next grid
                idx = move[j]-1
                point[0] += dx[idx]
                point[1] += dy[idx]
                point[2] += dz[idx]
                #find the move is a straight move or diagonal move
                count = 0
                if dx[idx] == 0:
                    count+=1
                if dy[idx] == 0:
                    count+=1 
                if dz[idx] == 0:
                    count+=1 
                if count == 2:
                    move_cost = 10
                else:
                    move_cost = 14
                if point[0] <= maze_size[0] and point[1] <= maze_size[1] and point[2] <= maze_size[2]:
                    #reject the visited node to avoid loop
                    if point not in visited:
                        path_cost += move_cost
                        point.append(move_cost)
                        path.append(point)
                        heapq.heappush(heap, (path_cost, path, steps))

    if curmin == sys.maxsize:
        f2 = open("output.txt", "w")
        f2.write("FAIL")
        f2.write('\n')
    f2.close()

def A_star(entry_point, exit_point, n, actions, maze_size):
    #print("The Function A_star begins")

    curmin = sys.maxsize
    #create a queue for A* search
    heap = []
    entry_point.append(0)
    heapq.heappush(heap, (0, 0, [entry_point], 1))
    #record the points that has been visited already
    visited = []

    #function to get h(n)
    def Heuristics(exit_point, grid):
        return abs(grid[0]-exit_point[0])+abs(grid[1]-exit_point[1])+abs(grid[1]-exit_point[1])

    while heap:
        tmp = heapq.heappop(heap)

        f = tmp[0]
        cost = tmp[1]
        cur_path = tmp[2][:]
        cur_point = cur_path[-1][0:3][:]
        steps = tmp[3]
        if cur_point in visited:
            continue
        visited.append(cur_point)

        if cur_point == exit_point:
            if cost < curmin:
                #update the current min cost
                curmin = cost

                f3 = open("output.txt", "w")           
                f3.write(str(cost))
                f3.write('\n')
                f3.write(str(steps))
                f3.write('\n')
                for j in range(len(cur_path)):
                    s = ' '.join([str(k) for k in cur_path[j]])
                    f3.write(s)
                    f3.write('\n')
                f3.close()

        i = 0
        while i<len(actions) and cur_point != actions[i][0:3]:
            i+=1
        if i < len(actions):
            move = actions[i][3:]
            #print('next action is', move)
            steps += 1
            for j in range(len(move)):
                point = cur_point[:]
                path_cost = cost
                move_cost = cost
                path = cur_path[:]
                cur_f = f
                #find the next grid
                idx = move[j]-1
                point[0] += dx[idx]
                point[1] += dy[idx]
                point[2] += dz[idx]
                #find the move is a straight move or diagonal move
                count = 0
                if dx[idx] == 0:
                    count+=1
                if dy[idx] == 0:
                    count+=1 
                if dz[idx] == 0:
                    count+=1 
                if count == 2:
                    move_cost = 10
                else:
                    move_cost = 14
                path_cost += move_cost
                cur_f = path_cost+Heuristics(exit_point, point)
                if point[0] <= maze_size[0] and point[1] <= maze_size[1] and point[2] <= maze_size[2]:
                    #reject the visited node to avoid loop
                    if point not in visited:
                        point.append(move_cost)
                        path.append(point)
                        heapq.heappush(heap, (cur_f, path_cost, path, steps))
    
    if curmin == sys.maxsize:
        f3 = open("output.txt", "w")
        f3.write("FAIL")
        f3.write('\n')
    f3.close()



#f = open("/Users/yueyuexu/Desktop/561/HW/hw1/input.txt")
f = open("input.txt")
method = f.readline().strip()
#print(method)
line = f.readline().strip()
maze_size = line.split(' ')
maze_size = list(map(int, maze_size))
#print('maze size is ', maze_size)

line = f.readline().strip()
entry_point= line.split(' ')
entry_point = list(map(int, entry_point))
#print('entry is ', entry_point)

line = f.readline().strip()
exit_point = line.split(' ')
exit_point = list(map(int,exit_point))
#print('exit is ', exit_point)

line = f.readline().strip()
n= int(line)
#print('number of actions is ', n)

actions = []
line = f.readline().strip()
while line:
    tmp = line.split(' ')
    tmp = list(map(int, tmp))
    actions.append(tmp)
    line = f.readline().strip()
#print('actions are ', actions)
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