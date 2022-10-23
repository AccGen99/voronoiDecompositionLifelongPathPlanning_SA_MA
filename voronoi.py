'''
Agents act as centres for voronoi cells
Cost function is energy drawn from batteries (default - length of path)
'''

import os
from statistics import stdev
import numpy as np
from scipy.spatial import Voronoi
from shapely.geometry import Polygon, Point
import matplotlib.pyplot as plt
import imageio

from parameters import *



class voronoi_centre:
    def __init__(self, centre, polygon, workload):
        self.centre = centre
        self.partition = polygon
        self.workload = workload



class voronoi_decomposer:
    def __init__(self, points):
        self.centres = points

        self.counts = 0
        self.frames = []

    def get_bounding_polygon(self, agent_centre):
        bounds = np.array([[0, 0], [1, 0], [1, 1], [0, 1]])
        regions = self.bounded_voronoi(bounds, self.centres)
        dists = [np.linalg.norm(self.centres[i]-agent_centre) for i in range(len(self.centres))]
        min_dist = np.min(dists)
        closest_centre_index = np.where(dists == min_dist)[0][0]
        poly_coords = regions[closest_centre_index]
        return poly_coords

    def bounded_voronoi(self, bnd, pnts):
        #Added 3 dummy mother points to make the Voronoi region of all mother points bounded
        gn_pnts = np.concatenate([pnts, np.array([[100, 100], [100, -100], [-100, 0]])])
        vor = Voronoi(gn_pnts)
        bnd_poly = Polygon(bnd)
        vor_polys = []
        
        #Repeat for non-dummy mother points
        for i in range(len(gn_pnts) - 3):
            vor_poly = [vor.vertices[v] for v in vor.regions[vor.point_region[i]]]
            i_cell = bnd_poly.intersection(Polygon(vor_poly))
            vor_polys.append(list(i_cell.exterior.coords[:-1]))
        return vor_polys

    def get_balanced_starts_voronoi(self):
        prev_centres = []

        # Collect infos about all centres for re-partitioning algorithm
        for centre in self.centres:
            a = voronoi_centre(centre, self.get_bounding_polygon(centre), Polygon(self.get_bounding_polygon(centre)).area)
            prev_centres.append(a)

        # Force-balance based voronoi re-partitioning
        iters = 0

        global ALPHA
        while iters < ITERS:
            ALPHA += 0.01*iters/ITERS
            iters += 1
    
            velocities = np.zeros((len(self.centres), 2))

            forces = self.get_virtual_forces(prev_centres)
            velocities = self.get_virtual_velocities(forces, prev_centres, velocities)
            for i in range(len(self.centres)):
                prev_centres[i].centre += velocities[i] * DEL_T

            workloads = self.get_workload_values(prev_centres)
            std_dev = stdev(workloads)

            if iters % 5 == 0:
                # print('std_dev - {:.3g}'.format(std_dev))
                if iters % 50 == 0:
                    print(str(iters*100/ITERS) + '%' + ' done!')
                # self.plot_centres(prev_centres, std_dev)

            self.check_within_bounds(prev_centres)

            if std_dev < STD_DEV_TOL:
                break

        # self.make_gif()

        new_centres = []
        for item in prev_centres:
            new_centres.append(item.centre)

        return np.array(new_centres)

    def check_within_bounds(self, centre_objects):
        boundary = Polygon(np.array([[0, 0], [1, 0], [1, 1], [0, 1]]))
        for item in centre_objects:
            if Point(item.centre).within(boundary):
                continue
            else:
                # self.make_gif()
                raise ValueError('Points out of bound')

    def make_gif(self):
        with imageio.get_writer('__{}centres.gif'.format(len(self.centres)), mode='I', duration=0.25) as writer:
            for frame in self.frames:
                image = imageio.imread(frame)
                writer.append_data(image)
        print('gif complete\n')

        # Remove files
        # for filename in self.frames[:-1]:
        #     os.remove(filename)

    def plot_centres(self, centre_objects, std_dev):
        for item in centre_objects:
            plt.scatter(item.centre[0], item.centre[1], color='green')
            partition = Polygon(item.partition)
            plt.plot(*partition.exterior.xy, color = 'blue')

        plt.suptitle('workload std_dev - {:.3g}'.format(std_dev))
        frame_name = 'balancing_' + str(self.counts) + '.png'
        self.counts += 1
        self.frames.append(frame_name)
        plt.savefig(frame_name)
        plt.clf()

    def get_workload_values(self, centre_objects):
        workloads = []
        for item in centre_objects:
            item.partition = self.get_bounding_polygon(item.centre)
            item.workload = Polygon(item.partition).area
            workloads.append(item.workload)
        return workloads

    def get_virtual_velocities(self, forces, centre_objects, velocities):
        viscous = np.zeros((len(self.centres), 2)) #np.random.rand(len(self.centres), 2)
        final_velocities = []
        for i in range(len(centre_objects)):
            velocity = velocities[i] 
            velocity += (forces[i] - LAMDA * viscous[i]) / (BETA * DEL_T)
            final_velocities.append(velocity)
        return final_velocities

    def get_virtual_forces(self, centre_objects):
        forces = []
        for i in range(len(centre_objects)):
            force = 0
            for j in range(len(centre_objects)):
                if i == j:
                    continue
                else:
                    unit_vector = (centre_objects[j].centre - centre_objects[i].centre) / np.linalg.norm(centre_objects[j].centre - centre_objects[i].centre)
                    force += ALPHA * (centre_objects[j].workload - centre_objects[i].workload) * unit_vector
            forces.append(force)
        return forces





if __name__ == '__main__':
    centres = np.random.rand(8,2)
    trial = voronoi_decomposer(centres)
    final_centres = trial.get_balanced_starts_voronoi()
