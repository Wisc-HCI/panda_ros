#!/usr/bin/env python
import numpy as np
import PyBSpline
import matplotlib.pyplot as plt
import math



################################################
# Calculates the score for a particular segment#
################################################
def score_segment(start,end,x,y):
    idx_start = np.searchsorted(x, start, side="left")
    idx_end = np.searchsorted(x, end, side="right")

    if(idx_start==idx_end):
        return np.inf

    mean = np.mean(y[idx_start:idx_end])
    max = np.max(y[idx_start:idx_end])

    # This will attempt to drive down the overall score, which
    # is easiest when the sections are evenly split
    score = (np.abs(max-mean))*np.power((end-start),1)
    return score

def score_set_of_supports(x,y,support_nodes):
    score = 0.0
    for ii in range(0,len(support_nodes)-1):
        #print(score_segment(support_nodes[ii],support_nodes[ii+1],x,y))
        score+=score_segment(support_nodes[ii],support_nodes[ii+1],x,y)
    return score


################################################
# Calculate piecewise constant approximation   #
################################################
def calculate_piecewise_constant_approx(x,y):

    support_nodes = np.array([0.0, 1.0])

    fxn_gain = 1.0
    gain_threshold = 0.05

    while(fxn_gain>gain_threshold):
        best_score = np.inf
        last_score = score_set_of_supports(x,y,support_nodes)
        best_node = -1.0

        for ii in np.arange(0,1,0.01):
            if ii not in support_nodes:
                support_nodes_temp = np.sort(np.append(support_nodes,ii))
                temp_score = score_set_of_supports(x,y,support_nodes_temp)

                #print("ii ",ii," ",temp_score)
                if(temp_score<best_score):
                    best_score = temp_score
                    best_node = ii

        print("BEST NODE: ",best_node," ",best_score)
        fxn_gain = last_score-best_score
        if(fxn_gain>gain_threshold):
            support_nodes = np.sort(np.append(support_nodes, best_node))

    print("SUPNODES:",support_nodes)
    return support_nodes

################################################
# Add smooth transitions to the support nodes  #
################################################
def create_smooth_variance_curve(support_nodes,x,y):
    output = np.zeros(np.size(y))
    for node_id in range(0, len(support_nodes) - 1):
        idx_start = np.searchsorted(x, support_nodes[node_id], side="left")
        idx_end = np.searchsorted(x, support_nodes[node_id + 1], side="right")
        max = np.max(y[idx_start:idx_end])

        output[idx_start:idx_end]=max

        if(node_id>0): # connect to previous
            transition_half_length = 0.00
            pre_transition = output[idx_start-1]
            transition = output[idx_start]
            time_tx = x[idx_start]

            # Set up a cubic polynomial for the interpolated values
            start_tx = np.searchsorted(x,time_tx-transition_half_length, side="left")
            end_tx = np.searchsorted(x, time_tx + transition_half_length, side="right")

            LHS=np.array([[1.0, start_tx, start_tx*start_tx, start_tx*start_tx*start_tx],
                      [0.0, 1.0, 2*start_tx, 3*start_tx*start_tx],
                      [1.0, end_tx, end_tx*end_tx, end_tx*end_tx*end_tx],
                      [0.0, 1.0, 2*end_tx, 3*end_tx*end_tx]])
            RHS = np.array([pre_transition, 0.0, transition, 0.0]).reshape((4,1))
            poly = np.linalg.solve(LHS,RHS)

            for ii in range(start_tx,end_tx):
                output[ii]=poly[0]+poly[1]*ii+poly[2]*ii*ii+poly[3]*ii*ii*ii

    return output

################################################
# Set up an example to test the scaling        #
################################################
def main():
    # # Plot 1
    # x = np.linspace(0,1,40)
    # y = np.array([0.91, 0.6, 0.7, 0.7, 0.83, 0.92, 0.42, 0.1, 0.8, 0.4,
    #               0.7, 0.9, 0.1, 0.8, 0.75, 0.46, 0.83, 0.4, 0.95, 0.85,
    #               0.5, 0.3, 0.1, 0.4, 0.32, 0.42, 0.15, 0.24, 0.35, 0.45,
    #               0.21, 0.16, 0.26, 0.35, 0.41, 0.06, 0.24, 0.01, 0.33, 0.38])

    # Plot 2
    x = np.linspace(0, 1.0)
    y = 0.3 * np.sin(30 * x) + 0.2 * np.sin(50 * x - np.pi / 6)
    y[0:15] += 2
    y[40:] += 1

    # # Plot 3
    # x = np.linspace(0, 1, 40)
    # y = np.array([0.4, 0.35, 0.2, 0.1, 0.34, 0.27, 0.15, 0.38, 0.4, 0.1,
    #               0.35, 0.17, 0.25, 0.75, 0.80, 0.75, 0.25, 0.1, 0.4, 0.31,
    #               0.4, 0.35, 0.2, 0.1, 0.34, 0.27, 0.15, 0.38, 0.4, 0.1,
    #               0.4, 0.35, 0.2, 0.1, 0.34, 0.27, 0.15, 0.38, 0.4, 0.1])

    plt.plot(x,y,color='blue')

    support_nodes = calculate_piecewise_constant_approx(x,y)
    output = create_smooth_variance_curve(support_nodes,x,y)

    plt.plot(x,output,color='orange')

    plt.show()

if __name__ == "__main__":
    main()




