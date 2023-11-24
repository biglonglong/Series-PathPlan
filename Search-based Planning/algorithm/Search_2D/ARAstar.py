'''
Anytime_Repairing_Astar (ARA*): 
Astar Comparison(weighted heuristic、incons): fasten searching suboptimal path, optimize suboptimal path.
Attention: low_var_epsilon_init、var_epsilon_step、improve_path_break_iteration、fair cost(weighted heuristic, which makes points-near-source have larger cost_total comparing points-near-goal, lead to over-optimize points-near-goal)
'''

import math
import heapq
import numpy as np

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + r"\..\..")
from map import Plotting
from map import Env

class arastar:
    def __init__(self, source, goal, var_epsilon, var_epsilon_step, iter_limitation, fair_cost):
        self.env = Env.env()
        self.obs = self.env.obs
        self.source = source
        self.goal = goal

        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.open_set = [] 
        self.visited = []
        self.close_set = []
        self.incons_set = [] 
        self.path = []

        self.explore_base = dict()
        self.explore_tree = dict()

        self.var_epsilon = var_epsilon
        self.var_epsilon_step = var_epsilon_step
        self.iter_limitation = iter_limitation
        self.fair_cost = fair_cost

    def cost_heuristic(self, point, heuristic_type = "euclidean"):
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
        x_change = end_x - current_x / max(abs(end_x - current_x),1)
        y_change = end_y - current_y / max(abs(end_y - current_y),1)       

        while(current_x != end_x or current_y != end_y):
            current_x += x_change
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
        self.explore_base[self.source] = 0
        self.explore_base[self.goal] = math.inf
        self.explore_tree[self.source] = self.source
        heapq.heappush(self.open_set,
                       (self.cost_total(self.source), self.source))

    def improve_path(self, flag):
        visited_each = []

        while self.open_set:
            cost_total_explore_point, explore_point = heapq.heappop(self.open_set)
            self.close_set.append(explore_point)

            if flag:
                if explore_point == self.goal:
                    break
            else:
                if cost_total_explore_point - self.iter_limitation>= self.explore_base[self.goal]:
                    break

            for neighbor in self.get_neighbor(explore_point):
                new_cost = self.explore_base[explore_point] + self.cost_neighbor(explore_point, neighbor)

                if neighbor not in self.explore_base:
                    self.explore_base[neighbor] = math.inf
                if new_cost < self.explore_base[neighbor]:
                    self.explore_base[neighbor] = new_cost
                    self.explore_tree[neighbor] = explore_point

                    if neighbor not in self.close_set:
                        heapq.heappush(self.open_set, 
                                       (self.cost_total(neighbor), neighbor))
                    else:
                        self.incons_set.append(neighbor)

                    visited_each.append(neighbor)     

        self.visited.append(visited_each)

    def update_var_epsilon(self):
        degree_convergence = float("inf")

        if self.open_set:
            degree_convergence = min(self.explore_base[pointpair[1]] + self.cost_heuristic(pointpair[1]) for pointpair in self.open_set)
        if self.incons_set:
            degree_convergence = min(degree_convergence,
                                min(self.explore_base[point] + self.cost_heuristic(point) for point in self.incons_set))

        return min(self.var_epsilon, (self.explore_base[self.goal] + 0.0) / degree_convergence)

    def searching(self):
        self.torrent()
        self.improve_path(True)
        self.path.append(self.extract_path())

        while self.update_var_epsilon() > 1:
            self.var_epsilon -= self.var_epsilon_step

            for point in self.incons_set:
                heapq.heappush(self.open_set, (self.cost_total(point), point))
            
            self.incons_set = []
            self.close_set = []
            
            self.improve_path(False)
            self.path.append(self.extract_path())

        return self.path, self.visited

def main():
    source = (5, 25)
    goal = (45, 5)

    # more iteration, more optimal path
    iter_limitation = 10

    # get points-near-source optimal path
    var_epsilon = 1.7
    var_epsilon_step = 0.2

    # make points in suboptimal path lower cost but I didn't
    fair_cost = 0.8   

    ARastar = arastar(source, goal, var_epsilon, var_epsilon_step, iter_limitation, fair_cost)
    plot = Plotting.plotting(source, goal)
    path, visited = ARastar.searching()
    plot.animation("Anytime_Repairing_Astar (ARA*)", path, "ARAstar", visited)


if __name__ == '__main__':
    main()

