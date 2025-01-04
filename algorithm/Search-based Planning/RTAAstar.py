'''
Real-time_Adaptive_Astar (RTAA*): loop N steps' local global-goal updated A*search --> better global quality, less calculation.
    Attention: 
        1. suitable N、movements(heuristic_updated is global, but local_goal is local in update_local_goal, which makes extracting local-path is difficult)
'''

import math
import heapq
import numpy as np
import matplotlib.pyplot as plt

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + r"\..\..")
from map import Plotting
from map import Env_Base

class rtaastar:
    def __init__(self, source, goal, N, movements):
        self.env = Env_Base.env()
        self.obs = self.env.obs
        self.source = source
        self.goal = goal

        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.open_set = [] 
        self.close_set = []
        self.explore_base = dict()

        self.explore_tree = dict()
        self.visited = []
        self.path = []

        self.table_heuristic = dict()
        self.N = N
        self.movements = movements

    def cost_heuristic(self, point, heuristic_type = "euclidean"):
        if point in self.obs:
            return math.inf

        goal = self.goal

        if heuristic_type == "euclidean":
            return math.hypot(goal[0] - point[0], goal[1] - point[1])
        else:
            return abs(goal[0] - point[0]) + abs(goal[1] - point[1])

    def cost_total(self, point):
        return self.explore_base[point] + self.table_heuristic[point]

    def get_neighbor(self, point):
        return [(point[0] + move[0], point[1] + move[1]) for move in self.motions]

    def is_collision(self, start, end):
        if start in self.obs or end in self.obs:
            return True
        
        current_x,current_y,end_x,end_y = start[0],start[1],end[0],end[1]
        x_change = (end_x - current_x) / max(abs(end_x - current_x),1)
        y_change = (end_y - current_y) / max(abs(end_y - current_y),1)  

        while(current_x != end_x and current_y != end_y):
            current_x += x_change
            current_y += y_change
            if (current_x,current_y) in self.obs:
                return True

        while(current_x != end_x):
            current_x += x_change
            if (current_x,current_y) in self.obs:
                return True
        while(current_y != end_y):
            current_y += y_change
            if (current_x,current_y) in self.obs:
                return True
            
        return False
        
    def cost_neighbor(self, start, end, neighbor_type = "diagonal"):
        if self.is_collision(start, end):
            return math.inf
        
        x_dis = abs(end[0] - start[0])
        y_dis = abs(end[1] - start[1])

        if neighbor_type == "diagonal":
            return x_dis + y_dis + (np.sqrt(2) - 2) * min(x_dis, y_dis)
        else:
            return x_dis + y_dis

    def extract_path_former(self, local_source, local_goal):
        path = [local_source]
        point_path = local_source
        movements = self.movements

        while movements:
            heuristic_neighbor = dict()
            for neighbor in self.get_neighbor(point_path):
                heuristic_neighbor[neighbor] = self.table_heuristic[neighbor]
    
            point_path = min(heuristic_neighbor, key=heuristic_neighbor.get)
            path.append(point_path)
            movements-=1

            if point_path == local_goal:
                break

        return point_path, list(path)

    def extract_path_final(self, local_source):
        path = [self.goal]
        point_path = self.goal

        while True:
            point_path = self.explore_tree[point_path]
            path.append(point_path)
            
            if point_path == local_source:
                break
        
        return list(reversed(path))

    def extract_path_combine(self):
        path = [(-1,-1)]
        for i in range(len(self.path)):
            path = path[:-2] + self.path[i]

        return path

    def torrent(self):
        for i in range(self.env.x_range):
            for j in range(self.env.y_range):
                self.table_heuristic[(i, j)] = self.cost_heuristic((i, j))       

    def update_local_goal(self):
        heuristic_updated = dict()
        
        local_goal = (-1, -1)
        minest_cost_total = math.inf
        for _, point in self.open_set:
            temp_cost_total = self.cost_total(point)
            if temp_cost_total < minest_cost_total:
                local_goal = point
                minest_cost_total = temp_cost_total

        for point in self.close_set:
            heuristic_updated[point] = minest_cost_total - self.explore_base[point]
            
        return local_goal, heuristic_updated            
 
    def searching(self):
        self.torrent()
        local_source = self.source

        while True:
            self.open_set = []
            self.close_set = []
            self.explore_base = {local_source: 0}
            self.explore_tree = {local_source: local_source}

            heapq.heappush(self.open_set, (self.cost_total(local_source), local_source))

            count = 0
            while self.open_set:
                count += 1

                _, explore_point = heapq.heappop(self.open_set)
                self.close_set.append(explore_point)

                if explore_point == self.goal:
                    local_path = self.extract_path_final(local_source)
                    self.path.append(local_path)
                    self.path.append(self.extract_path_combine())
                    self.visited.append(self.close_set)
                    self.visited.append(list())
                    return self.path, self.visited

                for neighbor in self.get_neighbor(explore_point):
                    new_cost = self.explore_base[explore_point] + self.cost_neighbor(explore_point, neighbor)

                    if neighbor not in self.explore_base:
                        self.explore_base[neighbor] = math.inf
                        self.explore_tree[neighbor] = None
                    if new_cost < self.explore_base[neighbor]:
                        self.explore_base[neighbor] = new_cost
                        self.explore_tree[neighbor] = explore_point
                        heapq.heappush(self.open_set, (self.cost_total(neighbor), neighbor))
                
                if count == self.N:
                    local_goal, heuristic_updated = self.update_local_goal()
                    for point_heuristic in heuristic_updated:
                        self.table_heuristic[point_heuristic] = heuristic_updated[point_heuristic]

                    local_arrival, local_path = self.extract_path_former(local_source, local_goal)
                    self.path.append(local_path)
                    self.visited.append(self.close_set)
                    local_source = local_arrival
                    break

def main():
    source = (5, 5)
    goal = (45, 25)

    # real-time interval
    N = 240

    # extracting points
    movements = 20

    RTaastar = rtaastar(source, goal, N, movements)
    plot = Plotting.plotting(source, goal)
    path, visited =RTaastar.searching()
    plot.animation("Real-time_Adaptive_Astar (RTAA*)", path, False, "RTAAstar", visited)
    
    plt.show()

if __name__ == '__main__':
    main()