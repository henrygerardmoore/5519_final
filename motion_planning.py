import matplotlib.pyplot as plt
import numpy as np
import map_load
import c_space
import time


class TreeNode:
    def __init__(self, location, parent):
        self.location = location
        self.children = []
        self.parent = parent


def v_direction(l1, l2):
    # returns a unit vector from l1 to l2
    return (l2 - l1) / distance(l1, l2)


def distance(p1, p2):
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
        while (configuration_space.any_obstacles_intersect_point(
                q_new) or configuration_space.any_obstacles_intersect_line(q_near, q_new)) and step > r * 0.01:
            step = step - 0.1 * r
            q_new = q_near + step * direction

        # now q_new is collision free or the same as q_near
        if np.array_equal(q_new, q_near) or step < r * 0.1:
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
            print("Goal found")
    if goal_not_found:
        print("Goal not found")

    # display_tree(tree_nodes[0], configuration_space)
    return path, path_length, time.time() - t0


def display_tree(tree_node, configuration_space):
    display_c_space(configuration_space)
    display_tree_helper(tree_node)
    plt.show()


def display_tree_helper(tree_node):
    for child in tree_node.children:
        plt.plot([tree_node.location[0], child.location[0]], [tree_node.location[1], child.location[1]], 'r')
        display_tree_helper(child)


def recover_path(tree_nodes):
    # goal node will be at end of tree node list
    path = [tree_nodes[-1].location]
    cur_node = tree_nodes[-1]
    while cur_node.parent is not None:
        cur_node = cur_node.parent
        path.append(cur_node.location)

    return path


def compute_length(path):
    cumulative_length = 0
    l0 = path[0]
    for loc in path[1:]:
        l1 = loc
        cumulative_length = cumulative_length + distance(l0, l1)
        l0 = l1

    return cumulative_length


def smooth_path(old_path, configuration_space):
    print("Smoothing")
    num_passes = 1
    path = old_path[:]
    for i in range(0, num_passes):
        index = 1
        while index < len(path) - 1:
            if not configuration_space.any_obstacles_intersect_line(path[index - 1], path[index + 1]):
                del (path[index])
            else:
                index = index + 1
    print("Done smoothing")
    return path


def generate_goal_matrix(goals, filename, n_samples=10000, r=0.3, p_goal=0.05, eps=0.2):
    [resolution, configuration_space] = map_load.load_map(filename)
    configuration_space = c_space.CSpace(configuration_space, goals, resolution)
    n = len(goals)
    goal_matrix = np.zeros([n, n], dtype=np.ndarray)
    index = 0
    for i in range(0, n):
        for j in range(0, n):
            index = index + 1
            if j >= i:
                continue

            start_loc = goals[i]
            goal_loc = goals[j]
            cur_path = goal_bias_rrt(n_samples, r, p_goal, eps, configuration_space, start_loc, goal_loc)[0]
            # cur_path = smooth_path(cur_path, configuration_space)
            goal_matrix[i, j] = cur_path
            if cur_path is not None:
                reverse_path = cur_path[::-1]
            else:
                reverse_path = None
            goal_matrix[j, i] = reverse_path
    return [configuration_space, goal_matrix]


def display_c_space(configuration_space):
    plt.imshow(configuration_space.c_space, cmap='gray',
               extent=[configuration_space.x_bounds[0], configuration_space.x_bounds[1],
                       configuration_space.y_bounds[0],
                       configuration_space.y_bounds[1]], origin='lower')


def display(configuration_space, goal_matrix):
    display_c_space(configuration_space)
    n = np.shape(goal_matrix)[0]
    for i in range(0, n):
        for j in range(0, n):
            if j >= i:
                continue
            display_path(goal_matrix[i, j])


def display_path(path):
    if path is None:
        return
    x_values = []
    y_values = []
    for point in path:
        x_values.append(point[0])
        y_values.append(point[1])
    plt.plot(x_values, y_values)


def main():
    # goal 1: 95, 65
    # goal 2: 48, 75
    # goal 3: 48, 55
    # goal 4: 70, 55
    # goal 5: 70, 75
    # goal 6: 60, 30
    # goal 7: 60, 100
    # goal 8: 20, 60

    goals = np.array([[3.25, 1], [2.7, 2.4], [3.8, 2.4], [3.8, 3.45], [2.7, 3.45], [3.25, 5.1], [5, 3], [1.5, 3]])
    [configuration_space, goal_matrix] = generate_goal_matrix(goals, 'tb_map.pgm')

    display(configuration_space, goal_matrix)
    plt.show()
    np.save('goal_matrix', goal_matrix)


if __name__ == '__main__':
    main()
