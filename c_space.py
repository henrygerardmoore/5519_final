import numpy as np
import math


class CSpace:
    def __init__(self, c_space, goals, resolution):
        self.black_value = 0
        self.gray_value = 205
        self.white_value = 254
        self.c_space = c_space
        self.goals = goals
        self.path_matrix = []
        w = np.shape(c_space)[1]
        h = np.shape(c_space)[0]
        self.shape = np.shape(c_space)
        self.x_bounds = [0, resolution * w]
        self.y_bounds = [0, resolution * h]
        self.resolution = resolution

    def any_obstacles_intersect_line(self, p1, p2):
        d = np.linalg.norm(p2 - p1)
        if d == 0:
            return self.any_obstacles_intersect_point(p1)
        v = (p2 - p1) / d
        num_points = np.ceil(d / self.resolution).astype(int) + 1
        for i in range(0, num_points + 1):
            cur_point = p1 + (float(i) / float(num_points)) * v * d
            if self.any_obstacles_intersect_point(cur_point):
                return True

        return False

    def any_obstacles_intersect_point(self, p):
        # check every adjacent pixel
        if not ((self.x_bounds[0] <= p[0] <= self.x_bounds[1]) and (self.y_bounds[0] <= p[1] <= self.y_bounds[1])):
            return True
        ind = self._loc_to_index(p)
        if ind[1] > 0 and self.c_space[ind[1] - 1, ind[0]] == self.black_value:
            return True
        if ind[1] < self.x_bounds[1] and self.c_space[ind[1] + 1, ind[0]] == self.black_value:
            return True
        if ind[0] > 0 and self.c_space[ind[1], ind[0] - 1] == self.black_value:
            return True
        if ind[0] < self.y_bounds[1] and self.c_space[ind[1], ind[0] + 1] == self.black_value:
            return True

        if ind[0] > 0 and ind[1] > 0 and self.c_space[ind[1] - 1, ind[0] - 1] == self.black_value:
            return True
        if ind[0] < self.y_bounds[1] and ind[1] < self.x_bounds[1] and self.c_space[
            ind[1] + 1, ind[0] + 1] == self.black_value:
            return True
        if ind[0] > 0 and ind[1] < self.x_bounds[1] and self.c_space[
            ind[1] + 1, ind[0] - 1] == self.black_value:
            return True
        if ind[1] > 0 and ind[0] < self.y_bounds[1] and self.c_space[
            ind[1] - 1, ind[0] + 1] == self.black_value:
            return True
        if self.c_space[ind[1], ind[0]] == self.black_value:
            return True
        return False

    def _index_to_loc(self, indices):
        return indices * self.resolution

    def _loc_to_index(self, loc):
        return np.floor(loc.astype(float) / self.resolution).astype(int)
