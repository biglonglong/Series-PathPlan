import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from map import Env_Base

class plotting:
    def __init__(self, source, goal):
        self.source, self.goal = source, goal
        self.env = Env_Base.env()
        self.obs = self.env.obs
        self.obs_dynamic = dict()

        self.ims = [[]]
        
    def update_obs_dynamic(self, obs):
        if obs not in self.obs_dynamic:
            self.obs_dynamic[obs] = "black"

        else:
            if self.obs_dynamic[obs] == "black":
                self.obs_dynamic[obs] = "white"
            else:
                self.obs_dynamic[obs] = "black"

    def plot_env(self, name):
        base_obs_x = [obs[0] for obs in self.env.obs]
        base_obs_y = [obs[1] for obs in self.env.obs]

        plt.title(name)
        plt.axis("equal")

        plt.plot(self.source[0], self.source[1], color="blue", marker="s")
        plt.plot(self.goal[0], self.goal[1], color="green", marker="s")
        plt.plot(base_obs_x, base_obs_y, "ks")

        plot_obs_dynamic = []
        for point,cl in self.obs_dynamic.items():
            plot_obs = plt.plot(point[0], point[1], color = cl, marker="s")
            plot_obs_dynamic = plot_obs_dynamic + plot_obs
    
        self.ims.append(self.ims[-1] + plot_obs_dynamic)
        plt.pause(1.0)

    def plot_visited(self, color_visited, *args):
        if self.source in args[0]:
            args[0].remove(self.source)
        if self.goal in args[0]:
            args[0].remove(self.goal)

        count = 0
        length = 40
        plot_explore_points = []

        if len(args) == 1:
            for point in args[0]:
                count += 1
                plot_explore_point = plt.plot(point[0], point[1], color=color_visited, marker='s')
                plot_explore_points = plot_explore_points + plot_explore_point

                if count % length == 0 or count == len(args[0]):
                    self.ims.append(self.ims[-1] + plot_explore_points)
                    plt.pause(0.01)
                    
        else:
            if self.source in args[1]:
                args[1].remove(self.source)
            if self.goal in args[1]:
                args[1].remove(self.goal)            

            len_visited_for, len_visited_back = len(args[0]), len(args[1])
            for i in range(max(len_visited_for, len_visited_back)):
                if i < len_visited_for:
                    count += 1
                    plot_explore_point_for = plt.plot(args[0][i][0], args[0][i][1], color=color_visited, marker='s')
                    plot_explore_points = plot_explore_points + plot_explore_point_for
                if i < len_visited_back:
                    count += 1
                    plot_explore_point_back = plt.plot(args[1][i][0], args[1][i][1], color=color_visited, marker='s')
                    plot_explore_points = plot_explore_points + plot_explore_point_back

                if count % length == 0 or count == len(args[0]+args[1]):
                    self.ims.append(self.ims[-1] + plot_explore_points)
                    plt.pause(0.01)

    def plot_path(self, color_path, path):
        path_x = [path[i][0] for i in range(len(path))]
        path_y = [path[i][1] for i in range(len(path))]
        
        plot_route = plt.plot(path_x, path_y, color=color_path, linewidth='2')
        self.ims.append(self.ims[-1] + plot_route)
        plt.pause(1.0)

    def animation(self, name, path, button = True, file="test", *args):
        plt.gcf().canvas.mpl_connect('key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
        
        self.plot_env(name)

        cl_v, cl_p = self.color_list()
        random_num = np.random.randint(0,len(cl_v))

        if len(args) == 2:
            self.plot_visited(cl_v[random_num], args[0], args[1])
            self.plot_path(cl_p[random_num], path)

        elif len(args) == 1 and len(args[0]) != 0 and isinstance(args[0][0], list):
            for k in range(len(path)):
                random_num = np.random.randint(0,len(cl_v))
                
                self.plot_visited(cl_v[random_num], args[0][k])
                self.plot_path(cl_p[random_num], path[k])    

        else:
            self.plot_visited(cl_v[random_num], args[0])
            self.plot_path(cl_p[random_num], path)     
        
        if button:
            ani = animation.ArtistAnimation(plt.gcf(), self.ims, interval=100,
                                                repeat_delay=1000, blit=True)
            ani.save(os.path.dirname(os.path.abspath(__file__)) + rf"\gif\{file}.gif",
                        writer="pillow")
        
    @staticmethod
    def color_list():
        cl_v = ['silver',
                'wheat',
                'lightskyblue',
                'royalblue',
                'slategray',
                'mediumpurple',
                'plum',]
        cl_p = ['gray',
                'orange',
                'deepskyblue',
                'red',
                'm',
                'purple',
                'magenta',]
        return cl_v, cl_p