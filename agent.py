import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon
import time

from parameters import *
from controller import min_ener_controller



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

        self.controller = min_ener_controller()

    def run_agent(self, coords_queue, part, env, target, routes_queue, targs_queue, energy_queue):
        '''
        coords_queue -> queue of target coordinates remaining to be reached
        env -> Copy of env for stepping from one point to another
        part -> Current partition of agent
        target -> First target, also acts as Voronoi centre of it's partition
        routes_queue -> To return back the followed route for plotting in plt
        '''
        routes = routes_queue.get()
        agent_route_obj = deepcopy(routes[self.ID])
        routes_queue.put(routes)
        agent_route_obj.add_partition(part)

        agent_targets = []

        # First step
        # print('Agent ID ' + str(self.ID) + ' running with initial energy - ' + str(self.energy))
        done, self.energy, wait_time = env.step_to_target(self.ID, self.current_node, target, C_TYPE, self.energy)
        agent_route_obj.add_node_time(target, wait_time)
        # self.visualize(self.current_node, target, self.ID)
        time.sleep(wait_time)
        self.target = None
        self.current_node = target

        self.targets_reached += 1

        while not done:
            # First find targets within self voronoi partition and remove them from unvisited list only if
            # no target is currently assigned
            if agent_targets == []:
                return_list = []
                uv_targs = coords_queue.get()
                if len(uv_targs) < 5:
                    # print('REGEN')
                    uv_targs = env.target_generator.generate_targets()
                # print(uv_targs)
                # print(type([]))
                # print(uv_targs == [])
                for i in range(len(uv_targs)):
                    if Point(uv_targs[i]).within(Polygon(part)):
                        agent_targets.append(uv_targs[i])
                    else:
                        return_list.append(uv_targs[i])
                coords_queue.put(np.array([return_list])[0])

            # Find minimum energy target within selected targets list and remove from list
            agent_targets, self.target = self.get_next_target(agent_targets, self.current_node)

            # Step to target
            done, self.energy, wait_time = env.step_to_target(self.ID, self.current_node, self.target, C_TYPE, self.energy)
            agent_route_obj.add_node_time(self.target, wait_time)
            self.current_node = self.target

            # queue_data = coords_queue.get()
            # print(queue_data)
            # print(newly_gen_target)
            # queue_data = np.append(queue_data, newly_gen_target)
            # queue_data = np.vstack((queue_data, newly_gen_target))
            # queue_data.reshape(-1, 2)
            # coords_queue.put(queue_data)

            self.targets_reached += 1
            time.sleep(wait_time)
            
            # if self.target is None:
            #     routes_queue = env.generate_new_targets(part, coords_queue)
            #     print('REGEN!')
            #     done = False
            # else:
            #     # self.visualize(self.current_node, target, self.ID)
            #     self.targets_reached += 1
            #     self.current_node = deepcopy(self.target)
            #     self.target = None

        print('Targets achieved by agent ID ' + str(self.ID) + ' - ' + str(self.targets_reached) + ' energy used - ' + str(self.energy0 - self.energy))
        val = targs_queue.get()
        val.append(self.targets_reached)
        targs_queue.put(val)

        val = energy_queue.get()
        val.append(self.energy0 - self.energy)
        energy_queue.put(val)

        final_routes = routes_queue.get()
        final_routes[self.ID] = deepcopy(agent_route_obj)
        # print(final_routes[0].visited)
        routes_queue.put(final_routes)

    def get_next_target(self, targets_list, current_node):
        costs = []
        for target in targets_list:
            costs.append(np.linalg.norm(target - current_node))

        min_energy_value = min(costs)
        min_ener_targ = targets_list[costs.index(min_energy_value)]
        targets_list = self.manual_remove(targets_list, min_ener_targ)
        return targets_list, min_ener_targ

    def manual_remove(self, arr, elem):
        new_arr = []
        for item in arr:
            if (item[0] == elem[0] and item[1] == elem[1]):
                continue
            else:
                new_arr.append(item)
        return new_arr
