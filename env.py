import numpy as np
from itertools import product
import multiprocessing as mp
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point

from parameters import *
from targets import targets2D
from voronoi import voronoi_decomposer
from controller import min_ener_controller
from agent import Agent

class env:
    def __init__(self):
        self.num_agents = NUM_AGENTS
        self.num_targets = NUM_TARGETS
        self.start = np.array([[0.0, 0.0]])
        self.targets = targets2D()
        self.current_targets = self.targets.generate_targets(num=NUM_AGENTS)
        self.decomposer = voronoi_decomposer(self.current_targets)
        self.energy = INITIAL_ENERGY
        self.agents = []
        self.counts = 0

    def run_agents(self):
        plt.xlim(0.0, 1.0)
        plt.ylim(0.0, 1.0)
        plt.figure(1)
        plt.axes((1.0, 0.0, 1.0, 1.0))

        agent_processes = []
        targets = self.targets.generate_targets()
        self.targets_queue = mp.Queue()
        self.targets_queue.put(targets)

        for i in range(self.num_agents):
            current_partition = self.decomposer.get_bounding_polygon(self.current_targets[i])
            a = Agent(self.energy, current_partition)
            self.agents.append(a)
            p = mp.Process(target=a.run_agent, args=(self.targets_queue, current_partition, self, self.current_targets[i]))
            p.start()
            agent_processes.append(p)

        for process in agent_processes:
            process.join()

    def add_grid(self):
        x1 = np.linspace(0, 1, RES)
        x2 = np.linspace(0, 1, RES)
        x1x2 = np.array(list(product(x1, x2)))
        self.grid = x1x2

    def step_to_target(self, agentID, from_node, to_node, controller_type, agent_rem_energy):
        if controller_type == 'None': # Euclidean distance as cost
            cost = np.linalg.norm(from_node - to_node)
        elif controller_type == 'min_energy': # Minimum energy TOMR system
            cost_function = min_ener_controller()
            cost = cost_function.get_energy_cons()

        rem_energy = agent_rem_energy - cost

        done = True if agent_rem_energy < cost else False
        # self.visualize(from_node, to_node, agentID)
        return done, rem_energy, cost

    def generate_new_targets(self, partition):
        targets = []

        while len(targets) < 5:
            new_targs = self.targets.generate_targets()
            for target in new_targs:
                if Point(target).within(Polygon(partition)):
                    targets.append(target)

        target_list = self.targets_queue.get()

        for i in range(len(targets)):
            target_list.append(targets[i])

        self.targets_queue.put(target_list)

    def visualize(self, from_node, to_node, ID):
        colours = ['lightcoral', 'firebrick', 'red', 'chocolate', 'peru', 'darkorange', 'darkgoldenrod', 'darkkhaki',
                'olive', 'greenyellow', 'forestgreen', 'turqupise', 'teal', 'cyan', 'deepskyblue', 'lightslategray',
                'royalblue', 'navy', 'indigo', 'darkorchid', 'plum', 'magenta', 'hotpink', 'pink']
        agent_color = colours[ID]

        x_vals = [from_node[0], to_node[0]]
        y_vals = [from_node[1], to_node[1]]

        plt.figure(1)
        plt.plot(x_vals, y_vals, color='red')
        # plt.scatter(x_vals, y_vals, agent_color)

        plt.savefig('test' + str(self.counts) + '.png')
        self.counts += 1





if __name__ == '__main__':
    env_obj = env()
    env_obj.run_agents()
