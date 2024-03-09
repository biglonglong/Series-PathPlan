"""
Anytime_Dstar (AD*): combination of ARAstar and LPAstar, ARAstar make LPAstar explore fast, but also distrub its rules(more point-near-source explored)
"""

import math
import heapq
import numpy as np
import matplotlib.pyplot as plt

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + r"\..\..")
from map import Plotting
from map import Env_Base

class adstar:
    def __init__(self, source, goal, var_epsilon, var_epsilon_step, iter_limitation, cost_gain, significant_standard):
        self.env = Env_Base.env()
        self.obs = self.env.obs
        self.source = source
        self.goal = goal

        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.open_set = []
        self.close_set = []
        self.incons_set = [] 
        self.g = dict()             # suitable cost_goal after setting obs
        self.rhs = dict()           # current cost_goal

        self.var_epsilon = var_epsilon
        self.var_epsilon_step = var_epsilon_step
        self.var_epsilon_top = var_epsilon

        self.iter_limitation = iter_limitation
        
        self.cost_gain = cost_gain

        self.count = 0
        self.significant_standard = significant_standard
        self.count_top = math.ceil(max(var_epsilon - 1.0, 0.0) / var_epsilon_step) + significant_standard

    def cost_heuristic(self, point, heuristic_type = "euclidean"):
        if point in self.obs:
            return math.inf

        goal = self.source

        if heuristic_type == "euclidean":
            return math.hypot(goal[0] - point[0], goal[1] - point[1])
        else:
            return abs(goal[0] - point[0]) + abs(goal[1] - point[1])

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
        length_top = 100

        path = [self.source]
        point_path = self.source

        while length_top:
            min_cost = math.inf
            close_neighbor = None

            for neighbor in self.get_neighbor(point_path):
                    cost_point_path = self.g[neighbor] + self.cost_neighbor(neighbor, point_path)
                    if cost_point_path < min_cost:
                        min_cost = cost_point_path
                        close_neighbor = neighbor

            point_path = close_neighbor
            path.append(point_path)

            if point_path == self.goal:
                break
            
            length_top -= 1

        return list(path)

    def torrent(self):
        for i in range(self.env.x_range):
            for j in range(self.env.y_range):
               self.g[(i, j)] = math.inf
               self.rhs[(i, j)] = math.inf     

        self.rhs[self.goal] = 0
        heapq.heappush(self.open_set, (self.calculate_key(self.goal), self.goal))

    def calculate_key(self, point):
        if self.g[point] > self.rhs[point]:
            return self.rhs[point] + self.var_epsilon * self.cost_heuristic(point), self.rhs[point]
        else:
            return self.g[point] + self.cost_heuristic(point), self.g[point]

    def update_rhs(self, point):   
        if point != self.goal:
            self.rhs[point] = min([math.inf] if point in self.obs else 
                                    [self.g[neighbor] + self.cost_neighbor(neighbor, point)
                                        for neighbor in self.get_neighbor(point)])
        
        for item in self.open_set:
            if item[1] == point:
                self.open_set.remove(item)
                break

        if self.g[point] != self.rhs[point]:
            if point not in self.close_set:
                heapq.heappush(self.open_set, (self.calculate_key(point), point))
            else:
                self.incons_set.append(point)
 
    def compute_shortest_path(self):
        while self.open_set:
            min_key, explore_point = heapq.heappop(self.open_set)

            if (min_key[0] - self.cost_gain - self.iter_limitation, min_key[1] - self.cost_gain - self.iter_limitation) >= \
                self.calculate_key(self.source) and self.g[self.source] == self.rhs[self.source]:
                break

            # confirm g after setting obs
            if self.g[explore_point] > self.rhs[explore_point]:  
                self.g[explore_point] = self.rhs[explore_point]
                self.close_set.append(explore_point)
            # recompute doubtful points
            else:
                self.g[explore_point] = math.inf       
                self.update_rhs(explore_point)   

            # spread rhs from neighbor
            for neighbor in self.get_neighbor(explore_point):
                self.update_rhs(neighbor)

    def plan(self):
        self.torrent()
        self.compute_shortest_path()

        return self.extract_path(), self.close_set

    def replan(self, changed_point):
        self.update_rhs(changed_point)

        for neighbor in self.get_neighbor(changed_point):
            self.update_rhs(neighbor)

        self.count += 1
        if self.count == self.count_top:
                self.count = 1
                self.var_epsilon = self.var_epsilon_top

        if self.var_epsilon > 1.0:
            self.var_epsilon -= self.var_epsilon_step

        for point in self.incons_set:
            heapq.heappush(self.open_set, (self.calculate_key(point), point))

        self.close_set = []
        self.incons_set = []
        self.compute_shortest_path()

    def on_press(self, event, plot):
        x, y = round(event.xdata), round(event.ydata)
        if x < 0 or x > self.env.x_range - 1 or y < 0 or y > self.env.y_range - 1:
            print("error area!")
        else:
            if (x, y) not in self.obs:
                self.obs.add((x, y))
                print("add obstacle at: ", (x, y))
            else:
                self.obs.remove((x, y))
                print("remove obstacle at: ", (x, y))
            plot.update_obs_dynamic((x, y))

            self.replan((x, y))

            plot.animation("Anytime_Dstar (AD*)", self.extract_path(), False, "ADstar", [])
            plt.gcf().canvas.draw_idle()

def main():
    source = (5, 5)
    goal = (45, 25)

    # get points-near-source optimal path
    var_epsilon = 1.8
    var_epsilon_step = 0.2

    # more iteration, more optimal path
    iter_limitation = 2.5

    # maximum difference between the new path and the previous path
    cost_gain = 2.5

    # significant edge cost changes
    significant_standard = 2

    ADstar = adstar(source, goal, var_epsilon, var_epsilon_step, iter_limitation, cost_gain, significant_standard)
    plot = Plotting.plotting(source, goal)
    path, visited = ADstar.plan()
    plot.animation("Anytime_Dstar (AD*)", path, False, "ADstar", visited)

    plt.gcf().canvas.mpl_connect('button_press_event',  lambda event: ADstar.on_press(event, plot=plot))
    plt.show()

if __name__ == '__main__':
    main()