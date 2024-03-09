'''
D*: plan by dijkstra from goal to source, move by explore_tree and if is_collision in dynamic map, replan by process_state, which former-search exit point until mink_ep >= h[collision_point]
    Attention: 
        1. fuse(cost_neighbor(point, new_obs) is math.inf)
        2. replan & on_press
            (1) process_state: former spread the new_obs_info along old_path, and find suitable neighbor as the path exits
            (2) mistaken exit: caused by historcal obs which didn't bolck historical path, one solution is serveral replan with path recorder, other solution is insert_obs every on_press
            (3) point_path: latter replan will modify explore_tree of former replan's points, path recorder need be in time
            (4) neighbor far or close: may be for speeding optimization, result may be not optimal and only for obs add
            (5) more debug, more comprehension
        3. break condition(mink_ep >= h[collision_point] prove that collision_point path is optimal)
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

class dstar:
    def __init__(self, source, goal):
        self.env = Env_Base.env()
        self.obs = self.env.obs
        self.source = source
        self.goal = goal

        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.open_set = []
        self.close_set = []
        self.t = dict()         # state
        self.h = dict()         # current_cost_goal
        self.k = dict()         # minest_cost_goal

        self.explore_tree = dict()

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
        path = [self.source]
        point_path = self.source

        while True:
            point_path = self.explore_tree[point_path]
            path.append(point_path)
            
            if point_path == self.goal:
                break
        
        return list(path)

    def torrent(self):
        for i in range(self.env.x_range):
            for j in range(self.env.y_range):
                self.t[(i ,j)] = 'NEW'
                self.h[(i ,j)] = math.inf
                self.k[(i ,j)] = math.inf  
                self.explore_tree[(i, j)] = None

        self.t[self.goal] = 'NEW'
        self.insert(self.goal, 0)
        self.explore_tree[self.goal] = self.goal

    def insert(self, point, new_h):
        if self.t[point] == 'NEW':
            self.k[point] = new_h
        elif self.t[point] == 'OPEN':
            self.k[point] = min(self.k[point], new_h)
        else:
            self.k[point] = min(self.h[point], new_h)

        self.h[point] = new_h

        if self.t[point] != 'OPEN':
            self.t[point] = 'OPEN'
            heapq.heappush(self.open_set, (self.k[point], point))

    def process_state(self):
        mink_ep, ep = heapq.heappop(self.open_set)
        self.close_set.append(ep)
        self.t[ep] = 'CLOSED'

        for neighbor in self.get_neighbor(ep):
            # close_neighbor find optimal path, change ep to close_neighbor
            if mink_ep < self.h[ep]:
                if self.h[neighbor] <= mink_ep and self.h[ep] > self.h[neighbor] + self.cost_neighbor(neighbor, ep):
                    self.explore_tree[ep] = neighbor
                    self.h[ep] = self.h[neighbor] + self.cost_neighbor(neighbor, ep)

            # exit point dijkstra optima
            # obs_start point cost_neighbor(stuck_point,obs) spread to h[stuck_point]
            if mink_ep == self.h[ep]:
                if (self.t[neighbor] == 'NEW') or \
                    (self.explore_tree[neighbor] != ep and self.h[neighbor] > self.h[ep] + self.cost_neighbor(neighbor, ep)) or \
                    (self.explore_tree[neighbor] == ep and self.h[neighbor] != self.h[ep] + self.cost_neighbor(neighbor, ep)):
                    self.explore_tree[neighbor] = ep
                    self.insert(neighbor, self.h[ep] + self.cost_neighbor(neighbor, ep))
            
            else:
                # cost_neighbor(stuck_point,stuck_point2) spread to h[stuck_point2]
                if (self.t[neighbor] == 'NEW') or \
                    (self.explore_tree[neighbor] == ep and self.h[neighbor] != self.h[ep] + self.cost_neighbor(neighbor, ep)):
                    self.explore_tree[neighbor] = ep
                    self.insert(neighbor, self.h[ep] + self.cost_neighbor(neighbor, ep))
                
                # current process_state -> close_neighbor find optimal path, ep exit point(dijkstra optima)
                elif self.explore_tree[neighbor] != ep and self.h[neighbor] > self.h[ep] + self.cost_neighbor(neighbor, ep):
                    self.insert(ep, self.h[ep])
                
                # former process_state -> close_neighbor find optimal path but no insert, far_neighbor exit point(dijkstra optima)
                elif self.explore_tree[neighbor] != ep and self.h[ep] > self.h[neighbor] + self.cost_neighbor(neighbor, ep) and self.t[neighbor] == 'CLOSED' and self.h[neighbor] > mink_ep:
                    self.insert(neighbor, self.h[neighbor])
                
                else:
                    pass

    def plan(self):
        self.torrent()        

        while True:
            self.process_state()
            if self.t.get(self.source) == 'CLOSED':
                break

        return self.extract_path(), self.close_set

    def replan(self, point_path):
        while True:
            self.process_state()

            if len(self.open_set) == 0:
                print("no path finded until no point explored")
                sys.exit(0)

            if self.open_set[0][0] >= self.h[point_path]:
                break

    def on_press(self, event, plot):
        x, y = round(event.xdata), round(event.ydata)
        if x < 0 or x > self.env.x_range - 1 or y < 0 or y > self.env.y_range - 1:
            print("error area!")
        else:
            if (x, y) not in self.obs:
                self.obs.add((x, y))
                plot.update_obs_dynamic((x, y))
                print("add obstacle at: ", (x, y))

                # solution 2: limited sacle obs spread
                self.insert((x, y), math.inf)
                self.explore_tree[(x, y)] = None
                self.process_state()

                path = [self.source]
                point_path = self.source

                try:
                    while point_path !=self.goal:
                        if self.is_collision(point_path, self.explore_tree[point_path]):
                            
                            # # solution 1: blocked point obs explore
                            # if self.t[point_path] == 'CLOSED':
                            #     self.insert(point_path, self.h[self.explore_tree[point_path]] + self.cost_neighbor(point_path, self.explore_tree[point_path])) 
   
                            self.replan(point_path)
                        else:
                            point_path = self.explore_tree[point_path]
                            path.append(point_path)
                except KeyError as e:
                    print(f"no path finded until explore {e}")
                    sys.exit(0)

                plot.animation("D*", path, False, "Dstar", [])
                plt.gcf().canvas.draw_idle()

def main():
    source = (5, 5)
    goal = (45, 25)

    DStar = dstar(source, goal)
    plot = Plotting.plotting(source, goal)
    path, visited = DStar.plan()
    plot.animation("D*", path, False, "Dstar", visited)

    plt.gcf().canvas.mpl_connect('button_press_event',  lambda event: DStar.on_press(event, plot=plot))
    plt.show()

if __name__ == '__main__':
    main()