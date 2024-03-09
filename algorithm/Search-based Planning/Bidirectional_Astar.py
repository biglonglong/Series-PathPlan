'''
Bidirectional_Astar: bi-direction A*search, until explore_tree connected --> half time-complexity
    Attention: 
        1. suit symmetrical env and cost_heuristic(asymmetrical env and cost_heuristic can lead the point_meet isn't the best path for the whole bi-half-env)
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

class bidirectional_astar:
    def __init__(self, source, goal):
        self.env = Env_Base.env()
        self.obs = self.env.obs
        self.source = source
        self.goal = goal

        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.open_set_for = []
        self.open_set_back = []
        self.close_set_for = []
        self.close_set_back = []
        self.explore_base_for = dict()
        self.explore_base_back = dict()
        
        self.explore_tree_for = dict()
        self.explore_tree_back = dict()

    def cost_heuristic(self, point, heuristic_type = "euclidean"):
        if point in self.obs:
            return math.inf

        goal = self.goal

        if heuristic_type == "euclidean":
            return math.hypot(goal[0] - point[0], goal[1] - point[1])
        else:
            return abs(goal[0] - point[0]) + abs(goal[1] - point[1])

    def cost_total_for(self, point):
        return self.explore_base_for[point] + self.cost_heuristic(point)

    def cost_total_back(self, point):
        return self.explore_base_back[point] + self.cost_heuristic(point)

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

    def extract_path(self, point_meet):
        path_for = [point_meet]
        point_path_for = point_meet
        while True:
            point_path_for = self.explore_tree_for[point_path_for]
            path_for.append(point_path_for)
            
            if point_path_for == self.source:
                break
        
        path_back = []
        point_path_back = point_meet
        while True:
            point_path_back = self.explore_tree_back[point_path_back]
            path_back.append(point_path_back)
            
            if point_path_back == self.goal:
                break 
        
        return list(reversed(path_for)) + list(path_back)

    def torrent(self):
        for i in range(self.env.x_range):
            for j in range(self.env.y_range):
                self.explore_base_for[(i, j)] = math.inf
                self.explore_tree_for[(i, j)] = None
                self.explore_base_back[(i, j)] = math.inf
                self.explore_tree_back[(i, j)] = None
        
        self.explore_base_for[self.source] = 0
        self.explore_tree_for[self.source] = self.source
        self.explore_base_back[self.goal] = 0
        self.explore_tree_back[self.goal] = self.goal       
        heapq.heappush(self.open_set_for, (self.cost_total_for(self.source), self.source))
        heapq.heappush(self.open_set_back, (self.cost_total_back(self.goal), self.goal))
     
    def searching(self):
        self.torrent()

        point_meet = self.source
        while self.open_set_for and self.open_set_back:
            _, explore_point_for = heapq.heappop(self.open_set_for)
            self.close_set_for.append(explore_point_for)
            
            if  self.explore_tree_back[explore_point_for]:  
                point_meet = explore_point_for
                break

            for neighbor_for in self.get_neighbor(explore_point_for):
                new_cost = self.explore_base_for[explore_point_for] + self.cost_neighbor(explore_point_for, neighbor_for)

                if new_cost < self.explore_base_for[neighbor_for]:
                    self.explore_base_for[neighbor_for] = new_cost
                    self.explore_tree_for[neighbor_for] = explore_point_for
                    heapq.heappush(self.open_set_for, (self.cost_total_for(neighbor_for), neighbor_for))

            _, explore_point_back = heapq.heappop(self.open_set_back)
            self.close_set_back.append(explore_point_back)
            
            if  self.explore_tree_for[explore_point_back]:
                point_meet = explore_point_back
                break

            for neighbor_back in self.get_neighbor(explore_point_back):
                new_cost = self.explore_base_back[explore_point_back] + self.cost_neighbor(explore_point_back, neighbor_back)

                if new_cost < self.explore_base_back[neighbor_back]:
                    self.explore_base_back[neighbor_back] = new_cost
                    self.explore_tree_back[neighbor_back] = explore_point_back
                    heapq.heappush(self.open_set_back, (self.cost_total_back(neighbor_back), neighbor_back))
    
        return self.extract_path(point_meet), self.close_set_for, self.close_set_back

def main():
    source = (5, 5)
    goal = (45, 25)

    BIdirectional_AStar = bidirectional_astar(source, goal)
    plot = Plotting.plotting(source, goal)
    path, visited_for, visited_back = BIdirectional_AStar.searching()
    plot.animation("Bidirectional_Astar", path, False, "Bidirectional_Astar", visited_for, visited_back)
    
    plt.show()

if __name__ == '__main__':
    main()
