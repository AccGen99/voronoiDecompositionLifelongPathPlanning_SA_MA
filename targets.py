import numpy as np
import matplotlib.pyplot as plt
from itertools import product
from shapely.geometry import Polygon, Point

from parameters import *

class targets2D:
    def __init__(self):
        self.targets = []
        self.successes = 0

    def generate_targets(self, num=0):
        if num == 0:
            num = NUM_TARGETS
        x1 = np.linspace(0, 1, RES)
        x2 = np.linspace(0, 1, RES)

        a = np.random.choice(x1, num, False).reshape(-1, 1)
        b = np.random.choice(x2, num, False).reshape(-1, 1)
        self.targets = np.hstack((a, b))
        return self.targets
    
    def plot_targets(self):
        plt.scatter(self.targets[:,0], self.targets[:,1], color='red')
        plt.savefig('trial.png')

if __name__ == '__main__':
    obj = targets2D()
    targs = obj.generate_targets()
    print(targs)
    obj.plot_targets()
