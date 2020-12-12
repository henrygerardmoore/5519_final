import matplotlib.pyplot as plt
import numpy as np
import map_load
import c_space
import time


class TreeNode:
    def __init__(self, location: np.array, parent):
        self.location = location
        self.children = []
        self.parent = parent


def v_direction(l1: np.array, l2: np.array):
    # returns a unit vector from l1 to l2
    return (l2 - l1) / distance(l1, l2)


def distance(p1: np.array, p2: np.array):
    return np.linalg.norm(p2 - p1)


def goal_bias_rrt(n, r, p_goal, eps, configuration_space, start_loc, goal_loc):
    t0 = time.time()
    width = configuration_space.x_bounds[1] - configuration_space.x_bounds[0]
    height = configuration_space.y_bounds[1] - configuration_space.y_bounds[0]
    x_min = configuration_space.x_bounds[0]
    y_min = configuration_space.y_bounds[0]
    start_node = TreeNode(start_loc, None)
    tree_nodes = [start_node]
    goal_not_found = True

    num_iter = 0
    path = None
    path_length = 0
    while goal_not_found and num_iter < n:
        num_iter = num_iter + 1
        if np.random.random_sample() < p_goal:
            q_rand = goal_loc
        else:
            q_rand = np.multiply(np.random.random(2), np.array([width, height])) + np.array([x_min, y_min])

        # find q_near
        min_dist = np.inf
        index_q_near = -1
        i = 0
        for node in tree_nodes:
            d = distance(node.location, q_rand)
            if d < min_dist:
                min_dist = d
                index_q_near = i
            i = i + 1

        q_near = tree_nodes[index_q_near].location
        direction = v_direction(q_near, q_rand)
        step = r
        q_new = q_near + step * direction
        while configuration_space.any_obstacles_intersect_point(
                q_new) or configuration_space.any_obstacles_intersect_line(q_near, q_new):
            step = step - 0.1 * r
            q_new = q_near + step * direction

        # now q_new is collision free or the same as q_near
        if np.array_equal(q_new, q_near):
            continue  # we just generate a new point

        tree_nodes.append(TreeNode(q_new, tree_nodes[index_q_near]))
        tree_nodes[index_q_near].children.append(tree_nodes[-1])

        if distance(goal_loc, q_new) < eps:
            # add goal node to path
            tree_nodes.append(TreeNode(goal_loc, tree_nodes[-1]))
            tree_nodes[-2].children.append(tree_nodes[-1])
            path = recover_path(tree_nodes)
            path_length = compute_length(path)
            goal_not_found = False

    return path, path_length, time.time() - t0


def recover_path(tree_nodes):
    # goal node will be at end of tree node list
    path = tree_nodes[-1].location
    cur_node = tree_nodes[-1]
    while cur_node.parent is not None:
        cur_node = cur_node.parent
        path.append(cur_node.location)

    path.reverse()
    return path


def compute_length(path):
    cumulative_length = 0
    l0 = path[0]
    for loc in path[1:]:
        l1 = loc
        cumulative_length = cumulative_length + distance(l0, l1)
        l0 = l1

    return cumulative_length


def main():
    n_samples = 1000
    r = 1
    p_goal = 0.05
    eps = 0.1
    [resolution, configuration_space] = map_load.load_map('tb_map.pgm')
    plt.imshow(c_space, cmap='gray')
    plt.show()
    # goal 1: 95, 65
    # goal 2: 48, 75
    # goal 3: 48, 55
    # goal 4: 70, 55
    # goal 5: 70, 75
    # goal 6: 60, 30
    # goal 7: 60, 100
    # goal 8: 20, 60

    goals = np.array([[95, 65], [48, 75], [48, 55], [70, 55], [70, 75], [60, 30], [60, 100], [20, 70]])

    configuration_space = c_space.CSpace(configuration_space, goals, resolution)
    n = len(goals)
    for i in range(0, n):
        for j in range(0, n):
            if j >= i:
                continue

            start_loc = goals[i]
            goal_loc = goals[j]
            goal_matrix = np.zeros([n, n])
            cur_path = goal_bias_rrt(n_samples, r, p_goal, eps, configuration_space, start_loc, goal_loc)
            goal_matrix[i, j] = cur_path
            reverse_path = cur_path.copy()
            reverse_path.reverse()
            goal_matrix[j, i] = reverse_path


if __name__ == '__main__':
    main()
