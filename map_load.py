import matplotlib.pyplot as plt
import numpy as np
import re
import yaml

# black is 0, grey is 192, white is 254
black_value = 0
gray_value = 205
white_value = 254


def read_pgm(filename, byteorder='>'):
    # code from stackoverflow user cgohlke at https://stackoverflow.com/questions/7368739/numpy-and-16-bit-pgm
    """Return image data from a raw PGM file as numpy array.

    Format specification: http://netpbm.sourceforge.net/doc/pgm.html

    """
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return np.frombuffer(buffer,
                         dtype='u1' if int(maxval) < 256 else byteorder + 'u2',
                         count=int(width) * int(height),
                         offset=len(header)
                         ).reshape((int(height), int(width)))


def crop_map(workspace):
    """
    :param workspace: 2D numpy array of map
    :return: new_map: map cropped to remove unused gray space
    """
    min_x = np.Inf
    min_y = np.Inf
    max_x = -np.Inf
    max_y = -np.Inf
    index_r = 0
    for row in workspace:
        if black_value in row:
            if index_r < min_y:
                min_y = index_r
            if index_r > max_y:
                max_y = index_r

            cur_min_x = np.where(row == black_value)[0]
            cur_max_x = cur_min_x[-1]
            cur_min_x = cur_min_x[0]

            if cur_min_x < min_x:
                min_x = cur_min_x
            if cur_max_x > max_x:
                max_x = cur_max_x

        index_r = index_r + 1
    if np.isinf(max_x) or np.isinf(min_x) or np.isinf(max_y) or np.isinf(min_y):
        raise (ValueError("Invalid Map"))

    return workspace[min_y:max_y, min_x:max_x]


def distance(loc1, loc2, resolution):
    return np.linalg.norm(loc1 - loc2) * resolution


def preprocess_c_space(workspace, w, h, radius, resolution):
    """
    removes unnecessary black pixels (ones that are not touched by white pixels)
    """
    c_space = np.copy(workspace)
    [num_added, c_space] = embiggen_c_space(c_space, radius, resolution)
    for i in range(1, w - 1):
        for j in range(1, h - 1):
            if workspace[i, j] == black_value:
                if white_value not in [workspace[i, j + 1], workspace[i + 1, j], workspace[i, j - 1],
                                       workspace[i - 1, j]]:
                    # if there are no white pixels adjacent, then we set this pixel to gray
                    c_space[i + num_added, j + num_added] = gray_value

    return c_space


def embiggen_c_space(c_space, radius, resolution):
    """
    makes the c space bigger so we can just 'stamp' black pixels on in a predefined manner to construct our c space
    more quickly
    """
    w = np.shape(c_space)[0]
    h = np.shape(c_space)[1]
    num_to_add = np.int(np.ceil(radius / resolution)) + 1
    c_space_new = np.zeros((w + 2 * num_to_add, h + 2 * num_to_add)) + gray_value
    c_space_new[num_to_add:(num_to_add + w), num_to_add:(num_to_add + h)] = c_space
    return [num_to_add, c_space_new]


def compute_stamp(radius, resolution):
    """
    computes the indices relative to the center that must be turned black
    """
    stamp = np.array([[0, 0]])
    radius_pixels = np.int(np.ceil(radius / resolution))
    center = np.array([[radius_pixels, radius_pixels]])
    for i in range(0, 2 * radius_pixels):
        for j in range(0, 2 * radius_pixels):
            if distance(center, np.array([i, j]), resolution) <= radius:
                stamp = np.append(stamp, np.array([[i, j]]) - center, 0)

    stamp_x = np.array([])
    stamp_y = np.array([])

    for index in stamp[1:-1]:
        stamp_x = np.append(stamp_x, index[0])
        stamp_y = np.append(stamp_y, index[1])

    stamp_x = stamp_x.astype('int')
    stamp_y = stamp_y.astype('int')
    return [stamp_x, stamp_y]


def generate_c_space(workspace, filename):
    yaml_filename = filename[0:-4] + '.yaml'
    with open(yaml_filename, 'r') as stream:
        data = (yaml.safe_load(stream))

    w = np.shape(workspace)[0]
    h = np.shape(workspace)[1]
    resolution = data['resolution']
    turtlebot_radius = 0.220
    FS = 1.5
    radius = turtlebot_radius * FS

    c_space = preprocess_c_space(workspace, w, h, radius, resolution)
    w = np.shape(c_space)[0]
    h = np.shape(c_space)[1]
    [stamp_x, stamp_y] = compute_stamp(radius, resolution)
    final_c_space = np.copy(c_space)
    for i in range(0, w):
        for j in range(0, h):
            if c_space[i, j] == black_value:
                final_c_space[i + stamp_x, j + stamp_y] = black_value

    return [resolution, final_c_space]


def load_map(filename):
    workspace = read_pgm(filename)
    workspace = crop_map(workspace)
    c_space = generate_c_space(workspace, filename)
    return c_space


def main():
    workspace = read_pgm('tb_map.pgm')
    # plt.imshow(workspace, cmap='gray')
    # plt.show()
    workspace = crop_map(workspace)
    # plt.imshow(workspace, cmap='gray')
    # plt.show()
    [resolution, c_space] = generate_c_space(workspace, 'tb_map.pgm')
    plt.imshow(c_space, cmap='gray')
    plt.show()


if __name__ == '__main__':
    main()
