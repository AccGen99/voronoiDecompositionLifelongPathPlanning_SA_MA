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
from router import route

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
        # plt.xlim(0.0, 1.0)
        # plt.ylim(0.0, 1.0)
        # plt.figure(1)
        # plt.axes((1.0, 0.0, 1.0, 1.0))

        agent_processes = []
        agent_routes = []
        targets = self.targets.generate_targets()
        self.targets_queue = mp.Queue()
        self.targets_queue.put(targets)
        self.agent_routes = mp.Queue()

        for i in range(self.num_agents):
            current_partition = self.decomposer.get_bounding_polygon(self.current_targets[i])
            a = Agent(self.energy, current_partition)

            route_obj = route(i, np.array([0.0, 0.0]))
            agent_routes.append(route_obj)

            self.agents.append(a)

            p = mp.Process(target=a.run_agent, args=(self.targets_queue, current_partition, self, self.current_targets[i], self.agent_routes))
            p.start()
            agent_processes.append(p)

        self.agent_routes.put(agent_routes)

        for process in agent_processes:
            process.join()

        all_routes = self.agent_routes.get()
        self.visualize(all_routes)

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

    def generate_new_targets(self, partition, generate_queue):
        targets = []

        while len(targets) < 5:
            new_targs = self.targets.generate_targets()
            for target in new_targs:
                if Point(target).within(Polygon(partition)):
                    targets.append(target)

        target_list = generate_queue.get()

        for i in range(len(targets)):
            target_list.append(targets[i])

        generate_queue.put(target_list)
        return generate_queue

    def visualize(self, data):
        plt.xlim(0.0, 1.0)
        plt.ylim(0.0, 1.0)
        # plt.figure(1)
        # plt.axes((1.0, 0.0, 1.0, 1.0))

        colours = ['lightcoral', 'firebrick', 'red', 'chocolate', 'peru', 'darkorange', 'darkgoldenrod', 'darkkhaki',
                'olive', 'greenyellow', 'forestgreen', 'turqupise', 'teal', 'cyan', 'deepskyblue', 'lightslategray',
                'royalblue', 'navy', 'indigo', 'darkorchid', 'plum', 'magenta', 'hotpink', 'pink']

        routes = []
        for i in range(len(data)):
            path = data[i]
            path = path.visited
            path = self.manual_reverse(path)
            routes.append(path)

        elses = 0
        step = 0
        while elses < len(data):
            elses = 0
            for j in range(len(data)):
                # print(routes[j])
                if routes[j] != [] and routes[j] is not None:
                    # line_to_plot = [routes[j][-1], routes[j][-2]]
                    # print('Path - ' + str(routes[j]))
                    # print(routes[j][-1][0])
                    # if len(routes[j]) == 1:
                    #     plt.scatter(routes[j][-1][0], routes[j][-1][1], color='black')
                    # else:
                    x_vals = [routes[j][-1][0], routes[j][-2][0]] #point_to_plot[0][0], point_to_plot[1][0]
                    y_vals = [routes[j][-1][1], routes[j][-2][1]]
                    point_to_plot = routes[j].pop()
                    plt.scatter(point_to_plot[0], point_to_plot[1], color='black')
                    plt.plot(x_vals, y_vals, color=colours[j])
                else:
                    elses += 1
                    continue
            plt.savefig('fig_' + str(step) + '.png')
            step += 1

    def manual_reverse(self, arr):
        reverse_arr = []
        length = len(arr)
        for i in range(length):
            reverse_arr.append(arr[-1])
            arr.pop()
        return reverse_arr





if __name__ == '__main__':
    env_obj = env()
    env_obj.run_agents()
