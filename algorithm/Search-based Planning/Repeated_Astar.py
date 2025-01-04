'''
Repeated_Astar: loop weighted cost_heuristic A*search, until var_epsilon < 1 --> suboptimal first optimal second
    Attention: 
        1. balance path search-speed and quality(repeat A* calculations with decreasing var_epsilon)
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

class repeated_astar:
    def __init__(self, source, goal, var_epsilon):
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

        self.var_epsilon = var_epsilon

    def cost_heuristic(self, point, heuristic_type = "euclidean"):
        if point in self.obs:
            return math.inf

        goal = self.goal

        if heuristic_type == "euclidean":
            return math.hypot(goal[0] - point[0], goal[1] - point[1])
        else:
            return abs(goal[0] - point[0]) + abs(goal[1] - point[1])

    def cost_total(self, point):
        return self.explore_base[point] + self.var_epsilon * self.cost_heuristic(point)

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

    def extract_path(self):
        path = [self.goal]
        point_path = self.goal

        while True:
            point_path = self.explore_tree[point_path]
            path.append(point_path)
            
            if point_path == self.source:
                break
        
        return list(path)

    def torrent(self):
        for i in range(self.env.x_range):
            for j in range(self.env.y_range):
                self.explore_base[(i, j)] = math.inf
                self.explore_tree[(i, j)] = None

        self.open_set = []
        self.close_set = []
        self.explore_base[self.source] = 0
        self.explore_tree[self.source] = self.source
        heapq.heappush(self.open_set, (self.cost_total(self.source), self.source))

    def searching(self):
        while self.var_epsilon >= 1:
            self.torrent()
            
            while self.open_set:
                _, explore_point = heapq.heappop(self.open_set)
                self.close_set.append(explore_point)
                
                if explore_point == self.goal:  
                    break

                for neighbor in self.get_neighbor(explore_point):
                    new_cost = self.explore_base[explore_point] + self.cost_neighbor(explore_point, neighbor)

                    if new_cost < self.explore_base[neighbor]:
                        self.explore_base[neighbor] = new_cost
                        self.explore_tree[neighbor] = explore_point
                        heapq.heappush(self.open_set, (self.cost_total(neighbor), neighbor))

            self.path.append(self.extract_path())
            self.visited.append(self.close_set)

            self.var_epsilon -= 0.5

        return self.path, self.visited

def main():
    source = (5, 5)
    goal = (45, 25)

    # search-speed and quality trade-off param
    var_epsilon = 2.5

    REpeated_astar = repeated_astar(source, goal, var_epsilon)
    plot = Plotting.plotting(source, goal)
    path, visited = REpeated_astar.searching()
    plot.animation("Repeated_Astar", path, False, "Repeated_Astar", visited)
    
    plt.show()

if __name__ == '__main__':
    main()