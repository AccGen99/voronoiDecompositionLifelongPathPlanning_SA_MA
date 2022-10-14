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
        self.current_targets = self.targets.generate_targets()
        self.agents = []

    def run_agents(self):
        plt.xlim(0.0, 1.0)
        plt.ylim(0.0, 1.0)
        plt.figure(1)
        plt.axes((1.0, 0.0, 1.0, 1.0))

        agent_processes = []

        for i in range(self.num_agents):
            a = Agent()
            self.agents.append(a)
            p = mp.Process(target=a.run_agent, args=())
            p.start()
            agent_processes.append(p)

        for process in agent_processes:
            process.join()

    def add_grid(self):
        x1 = np.linspace(0, 1, RES)
        x2 = np.linspace(0, 1, RES)
        x1x2 = np.array(list(product(x1, x2)))
        self.grid = x1x2

    def step_to_target(self, agentID, from_node, to_node, controller_type):
        if controller_type == 'None': # Euclidean distance as cost
            cost = np.linalg.norm(from_node - to_node)
        elif controller_type == 'min_energy': # Minimum energy TOMR system
            cost_function = min_ener_controller()
            cost = cost_function.get_energy_cons() #TODO
        # TODO

        rem_ener = self.agents[agentID].current_energy_level.get()
        self.agents[agentID].current_energy_level.put(rem_ener - cost)

        done = True if rem_ener > cost else False
        return done, rem_ener - cost, cost_function
