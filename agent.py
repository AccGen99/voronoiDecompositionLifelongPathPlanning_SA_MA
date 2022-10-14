'''
Plans using RIG_tree variant, basically RRT_star with info-node association
'''

import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon

from parameters import *
from voronoi import voronoi_decomposer

class Agent:
    num_agents = 0

    def __init__(self, start, energy, target_coords):
        self.ID = Agent.num_agents
        Agent.num_agents += 1

        self.current_node = start #env.start_points[self.ID]
        self.energy = energy #env.budgets[self.ID]
        self.target = target_coords
        self.energy0 = deepcopy(self.energy)
        self.targets_reached = 0

    def run_agent(self, coords_queue, ua_targ_list, env):
        from_node = self.current_node
        to_node = self.target

        print('Agent ' + str(self.ID) + ' running with initial energy - ' + str(self.energy))
        done, rem_ener, cost_f = env.step_to_target(self.ID, self.current_node, self.target, C_TYPE)

        self.targets_reached += 1

        while not done:
            print('Targets achieved for agent ' + str(self.ID) + ' - ' + str(self.target))

            self.visualize_trajectory(from_node, to_node)
            self.current_node = self.target

            kew = coords_queue.get()
            kew[self.ID] = deepcopy(self.current_node)
            coords_queue.put(kew)

            self.energy = rem_ener
            self.target = self.get_target(kew, ua_targ_list, cost_f)
            done, rem_ener, cost_f = env.step_to_target(self.ID, self.current_node, self.target, C_TYPE)
            self.targets_reached += 1

        print('Targets achieved by agent ID ' + str(self.ID) + ' - ' + str(self.targets_reached))

    def get_target(self, coords_q, ua_targets_list, cost_f):
        assignent = False

        centres = coords_q
        decomposer = voronoi_decomposer(centres)
        all_polys = []

        for centre in centres:
            all_polys.append(decomposer.get_bounding_polygon(centre))

        while not assignent:
            self_poly = Polygon(all_polys[self.ID])
            ua_dict = ua_targets_list.get()
            for i in range(len(ua_dict)):
                if Point(ua_dict[i]).within(self_poly):

    def visualize_trajectory(self, graph):
        plt.figure(1)#self.ID)
        centres = self.voronoi.centres
        vor = Voronoi(centres)
        voronoi_plot_2d(vor, show_vertices=False)

        x_vals = self.node_coords[:,0]
        y_vals = self.node_coords[:,1]

        edge_dict = graph.edges

        colors = ['indianred', 'peru', 'gold', 'olivedrab', 'lime', 'turquoise', 'aqua', 'slategrey', 'cornflowerblue', 
                    'darkorchid', 'fuchsia', 'hotpink', 'lightpink']
        if 1:
            for each_node in graph.nodes:
                connected_edges = edge_dict[str(each_node)]
                for node, edge in connected_edges.items():
                    if node != each_node:
                        x_e = [x_vals[int(each_node)], x_vals[int(node)]]
                        y_e = [y_vals[int(each_node)], y_vals[int(node)]]
                        plt.plot(x_e, y_e, color=colors[self.ID])

        plt.scatter(x_vals[0], y_vals[0], color = 'orange') # Start node, in orange
        plt.xlim(0.0, 1.0)
        plt.ylim(0.0, 1.0)

        path = '/home/vashisth/NUS/DomDeco_v2/max_budget__1_0/'
        plt.suptitle('Max_Budget = 2.5, 8 agents')
        plt.savefig(path + '_trial7__8Agents__' + str(self.ID) + '.png')
