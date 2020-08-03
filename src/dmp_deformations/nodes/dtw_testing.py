#!/usr/bin/env python

""" Tests DTW libraries
 Created: 03/20/2020
"""

__author__ = "Mike Hagenow"

import matplotlib.pyplot as plt
import numpy as np
from dtw import dtw

def main():

    x = np.array([0, 0, 0, 0, 1, 2, 3, 2, 1, 0]).reshape(-1, 1)
    y = np.array([0, 0, 0, 0.5, 1, 1.5, 2, 2.5, 3, 2.5, 2.0, 1.5, 1.0, 0.5, 0.0]).reshape(-1, 1)

    # x = []
    # y = []
    #
    # x.append((0,1,2))
    # x.append((1,2,3))
    # y.append((1,2,3))
    # x.append((2, 3, 4))
    # y.append((2, 3, 4))
    # x.append((3, 4, 5))
    # y.append((3, 4, 5))
    # x.append((4, 5, 6))
    # y.append((4, 5, 6))
    # x.append((5, 6, 7))
    # y.append((5, 6, 7))


    manhattan_distance = lambda x, y: np.abs(x - y)

    d, cost_matrix, acc_cost_matrix, path = dtw(x, y, dist=manhattan_distance)

    print(path[0])
    print(path[1])
    seg_index = 6
    print(path[1][int(np.round(np.median(np.where(path[0]==seg_index))))])
    plt.plot(x[path[0]])
    plt.plot(y[path[1]])
    plt.show()

if __name__ == "__main__":
    main()




