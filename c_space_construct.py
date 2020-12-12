import matplotlib.pyplot as plt
import numpy as np
import re


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


def crop_map(map):
    """
    :param map: 2D numpy array of map
    :return: new_map: map cropped to remove unused gray space
    """
    black_value = 0
    w = np.shape(map)[0]
    h = np.shape(map)[1]
    min_x = np.Inf
    min_y = np.Inf
    max_x = -np.Inf
    max_y = -np.Inf
    index_r = 0
    for row in map:
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
        raise(ValueError("Invalid Map"))

    return map[min_y:max_y,min_x:max_x]


def main():
    map = read_pgm('tb_map.pgm')
    plt.imshow(map, cmap='gray')
    plt.show()
    # black is 0, grey is 192, white is 254
    map = crop_map(map)
    plt.imshow(map, cmap='gray')
    plt.show()


if __name__ == '__main__':
    main()
