import numpy as np


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
        self.x_bounds = [0, resolution * w]
        self.y_bounds = [0, resolution * h]
        self.resolution = resolution

    def any_obstacles_intersect_line(self, p1, p2):
        pass

    def any_obstacles_intersect_point(self, p):
        pass

    def index_to_loc(self, indices: np.array):
        return indices * self.resolution

    def loc_to_index(self, loc: np.aray):
        return loc / self.resolution
