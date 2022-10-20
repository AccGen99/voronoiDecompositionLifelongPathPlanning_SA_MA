import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon
import time

from parameters import *

class Agent:
    num_agents = 0

    def __init__(self, energy, initial_partition):
        '''
        Agent class
        Takes input as first target and generates own voronoi partition with first target as
            voronoi centre. Stays in it's own partition unless everyone decides to 
            re-partition based on their remaining energies
        '''

        self.ID = Agent.num_agents
        Agent.num_agents += 1

        self.current_node = np.array([0.0, 0.0]) # Start point
        self.energy = energy
        self.section = initial_partition
        self.energy0 = deepcopy(self.energy)
        self.targets_reached = 0
        self.counts = 0

    def run_agent(self, coords_queue, part, env, target):
        '''
        coords_queue -> queue of target coordinates remaining to be reached
        env -> Copy of env for stepping from one point to another
        part -> Current partition of agent
        target -> First target, also acts as Voronoi centre of it's partition
        '''

        # First step
        print('Agent ID ' + str(self.ID) + ' running with initial energy - ' + str(self.energy))
        done, self.energy, wait_time = env.step_to_target(self.ID, self.current_node, target, C_TYPE, self.energy)
        self.visualize(self.current_node, target, self.ID)
        time.sleep(wait_time)
        self.target = target

        self.targets_reached += 1

        while not done:
            print('next target for agent ' + str(self.ID) + ' - ' + str(self.target))
            uv_targs = coords_queue.get() # Get list of unvisited targets
            new_targs = []

            for i in range(len(uv_targs)): # Find one target to capture
                if Point(uv_targs[i]).within(Polygon(part)):
                    self.target = uv_targs[i]
                    break

            for j in range(len(uv_targs)): # Find one target to capture
                if j == i:
                    pass
                else:
                    new_targs.append(uv_targs[j])

            coords_queue.put(new_targs) # Return back list for others to visit

            if self.target is None:
                env.generate_new_targets(part)
                done = False
            else:
                done, self.energy, wait_time = env.step_to_target(self.ID, self.current_node, self.target, C_TYPE, self.energy)
                time.sleep(wait_time)
                self.visualize(self.current_node, target, self.ID)
                self.targets_reached += 1
                self.current_node = deepcopy(self.target)
                self.target = None

        print('Targets achieved by agent ID ' + str(self.ID) + ' - ' + str(self.targets_reached))

    def visualize(self, from_node, to_node, ID):
        colours = ['lightcoral', 'firebrick', 'red', 'chocolate', 'peru', 'darkorange', 'darkgoldenrod', 'darkkhaki',
                'olive', 'greenyellow', 'forestgreen', 'turqupise', 'teal', 'cyan', 'deepskyblue', 'lightslategray',
                'royalblue', 'navy', 'indigo', 'darkorchid', 'plum', 'magenta', 'hotpink', 'pink']
        agent_color = colours[ID]

        x_vals = [from_node[0], to_node[0]]
        y_vals = [from_node[1], to_node[1]]

        plt.xlim(0.0, 1.0)
        plt.ylim(0.0, 1.0)
        plt.figure(self.ID)
        # plt.axes((1.0, 0.0, 1.0, 1.0))

        plt.plot(x_vals, y_vals, color=agent_color)
        # plt.scatter(x_vals, y_vals, agent_color)

        plt.savefig('test' + str(ID) + str(self.counts) + '.png')
        self.counts += 1

    # def visualize_trajectory(self, graph):
    #     plt.figure(1)#self.ID)
    #     centres = self.voronoi.centres
    #     vor = Voronoi(centres)
    #     voronoi_plot_2d(vor, show_vertices=False)

    #     x_vals = self.node_coords[:,0]
    #     y_vals = self.node_coords[:,1]

    #     edge_dict = graph.edges

    #     colors = ['indianred', 'peru', 'gold', 'olivedrab', 'lime', 'turquoise', 'aqua', 'slategrey', 'cornflowerblue', 
    #                 'darkorchid', 'fuchsia', 'hotpink', 'lightpink']
    #     if 1:
    #         for each_node in graph.nodes:
    #             connected_edges = edge_dict[str(each_node)]
    #             for node, edge in connected_edges.items():
    #                 if node != each_node:
    #                     x_e = [x_vals[int(each_node)], x_vals[int(node)]]
    #                     y_e = [y_vals[int(each_node)], y_vals[int(node)]]
    #                     plt.plot(x_e, y_e, color=colors[self.ID])

    #     plt.scatter(x_vals[0], y_vals[0], color = 'orange') # Start node, in orange
    #     plt.xlim(0.0, 1.0)
    #     plt.ylim(0.0, 1.0)

    #     path = '/home/vashisth/NUS/DomDeco_v2/max_budget__1_0/'
    #     plt.suptitle('Max_Budget = 2.5, 8 agents')
    #     plt.savefig(path + '_trial7__8Agents__' + str(self.ID) + '.png')
