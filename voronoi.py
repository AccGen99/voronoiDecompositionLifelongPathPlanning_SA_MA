'''
Agents act as centres for voronoi cells
Cost function is energy drawn from batteries (default - length of path)
'''

import numpy as np
from scipy.spatial import Voronoi
from shapely.geometry import Polygon, Point



class voronoi_decomposer:
    def __init__(self, points):
        self.centres = points

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
