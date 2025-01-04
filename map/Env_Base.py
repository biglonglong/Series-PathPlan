
class env:
    def __init__(self):
        self.x_range = 51
        self.y_range = 31
        self.obs = self.obs_map()

    def obs_map(self):
        x = self.x_range
        y = self.y_range
        obs = set()

        for i in range(x):
            obs.add((i, 0))
            obs.add((i, y - 1))      
        for i in range(y):
            obs.add((0, i))
            obs.add((x - 1, i))

        for i in range(10, 21):
            obs.add((i, 15))
        for i in range(15):
            obs.add((20, i)) 
        for i in range(15, 30):
            obs.add((30, i))
        for i in range(16):
            obs.add((40, i))

        return obs