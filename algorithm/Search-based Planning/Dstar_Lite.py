"""
D*_Lite: combination of Dstar and LPAstar,explore path with changing start constantly, new-obs only effect point between new-obs and goal, repair key of cost_heuristic(km) to make prioritization-explore these points
# check #
km what?
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

class dstar_lite:
    def __init__(self, source, goal, cost_gain):
        self.env = Env_Base.env()
        self.obs = self.env.obs
        self.source = source
        self.goal = goal

        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.open_set = []
        self.close_set = []
        self.g = dict()             # suitable cost_source after setting obs
        self.rhs = dict()           # current cost_source

        self.cost_gain = cost_gain
        self.start = source
        self.km = 0

    def cost_heuristic(self, point, start, heuristic_type = "euclidean"):
        if point in self.obs:
            return math.inf

        goal = start

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
        return min(self.g[point], self.rhs[point]) + self.cost_heuristic(point, self.start) + self.km, min(self.g[point], self.rhs[point])

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
            heapq.heappush(self.open_set, (self.calculate_key(point), point))
 
    def compute_shortest_path(self):
        while self.open_set:
            mink_ep, explore_point = heapq.heappop(self.open_set)
            self.close_set.append(explore_point)

            if (mink_ep[0] - self.cost_gain, mink_ep[1] - self.cost_gain) >= self.calculate_key(self.start) and \
                self.g[self.start] == self.rhs[self.start]:
                break

            # repair key of cost_heuristic(km)
            if mink_ep < self.calculate_key(explore_point):   
                heapq.heappush(self.open_set, (self.calculate_key(explore_point), explore_point))

            else:
                # confirm g after setting obs
                if self.g[explore_point] > self.rhs[explore_point]:  
                    self.g[explore_point] = self.rhs[explore_point]
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
        for neighbor in self.get_neighbor(changed_point):
            self.update_rhs(neighbor)

        self.compute_shortest_path()

    def on_press(self, event, plot):
        x, y = round(event.xdata), round(event.ydata)
        if x < 0 or x > self.env.x_range - 1 or y < 0 or y > self.env.y_range - 1:
            print("error area!")
        else:
            if (x, y) not in self.obs:
                self.obs.add((x, y))
                self.g[(x, y)] = math.inf
                self.rhs[(x, y)] = math.inf
                print("add obstacle at: ", (x, y))
            else:
                self.obs.remove((x, y))
                self.update_rhs((x, y))
                print("remove obstacle at: ", (x, y))
            plot.update_obs_dynamic((x, y))

            self.start = self.source
            self.km = 0

            cur_point_path = self.start
            path = [self.start]
            length_top = 100

            while cur_point_path != self.goal and length_top:
                min_cost = math.inf
                close_neighbor = None

                for neighbor in self.get_neighbor(cur_point_path):
                        
                        cost_point_path = self.g[neighbor] + self.cost_neighbor(neighbor, cur_point_path)
                        if cost_point_path < min_cost:
                            min_cost = cost_point_path
                            close_neighbor = neighbor

                        if neighbor == (x, y):
                            self.km += self.cost_heuristic(self.start, cur_point_path)
                            self.start = cur_point_path
                            self.replan((x, y))
                
                cur_point_path = close_neighbor
                path.append(cur_point_path)
                length_top -= 1

            plot.animation("D*_Lite", path, True, "Dstar_Lite", [])
            plt.gcf().canvas.draw_idle()
                
def main():
    source = (5, 5)
    goal = (45, 25)

    # maximum difference between the new path and the previous path
    cost_gain = 2.5

    DStar_LIte = dstar_lite(source, goal, cost_gain)
    plot = Plotting.plotting(source, goal)
    path, visited = DStar_LIte.plan()
    plot.animation("D*_Lite", path, False, "Dstar_Lite", visited)

    plt.gcf().canvas.mpl_connect('button_press_event',  lambda event: DStar_LIte.on_press(event, plot=plot))
    plt.show()

if __name__ == '__main__':
    main()