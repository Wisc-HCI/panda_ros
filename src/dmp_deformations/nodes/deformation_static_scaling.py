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

        print("BEST NODE: ",best_node," ",last_score-best_score)
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

    # # Plot 2
    # x = np.linspace(0, 1.0)
    # y = 0.3 * np.sin(30 * x) + 0.2 * np.sin(50 * x - np.pi / 6)
    # y[0:15] += 2
    # y[40:] += 1

    # # Plot 3
    # x = np.linspace(0, 1, 40)
    # y = np.array([0.4, 0.35, 0.2, 0.1, 0.34, 0.27, 0.15, 0.38, 0.4, 0.1,
    #               0.35, 0.17, 0.25, 0.75, 0.80, 0.75, 0.25, 0.1, 0.4, 0.31,
    #               0.4, 0.35, 0.2, 0.1, 0.34, 0.27, 0.15, 0.38, 0.4, 0.1,
    #               0.4, 0.35, 0.2, 0.1, 0.34, 0.27, 0.15, 0.38, 0.4, 0.1])

    # Plot 4 - real force variance data from wiping
    x = np.linspace(0,1,199)
    y = np.array([6.747283615016177, 7.232339501411629, 8.082689249814102, 8.891233161277691, 9.46633590541735,
                  8.530487527199337, 5.684581458767814, 4.442920650236722, 5.069355407282715, 4.471254519714687,
                  3.994041599342678, 3.497470877690367, 2.847738348853499, 2.7195217683680464, 2.8270482621572186,
                  2.902888607536033, 3.187501967461967, 3.6100147246371335, 3.6687755867015457, 3.3875169439075563,
                  2.8791154916601727, 2.457713567916587, 2.301288221025355, 2.5004246321555232, 2.7527717886375833,
                  2.6415166751473786, 2.4302260793071273, 2.3220837822438187, 2.2185199014439116, 2.068367870745392,
                  2.052153179812841, 2.219694719848755, 2.334806731669453, 2.4304726564672245, 2.602606251088901,
                  2.799111480501866, 3.0984090500665844, 3.5884179991273686, 3.83998367423314, 3.8590179431897425,
                  4.096621471634644, 4.366353999878428, 4.249074099572421, 3.8776982916113596, 4.074970097710618,
                  4.595954955693107, 4.665312779698405, 4.615780713206397, 4.6989617995028485, 5.119280449883843,
                  5.317498725681129, 4.953552222572823, 4.29050760219734, 4.371301819861945, 5.092303415028787,
                  5.1436919411492825, 4.9372294494653195, 5.077503103657717, 5.226245973411952, 5.091513359160661,
                  4.926273758881456, 4.9151535064909515, 5.212989273560411, 5.352317628714673, 4.6263279964929565,
                  4.416636416797696, 4.826995704221569, 4.801443107575831, 4.68103725702274, 4.580151864497108,
                  4.335102770312959, 4.2823999042249214, 4.237128808511945, 4.345566770200856, 4.683173363144457,
                  4.8018092453137475, 4.817587972272552, 4.627303487903922, 4.326653140947357, 4.228257878081008,
                  4.152520094161525, 4.012522168860383, 3.9329463957186888, 3.972487200482192, 3.8397672736528627,
                  3.50423526278083, 3.1554207843174904, 3.0449524029719064, 3.1722449717845773, 3.187555006255952,
                  3.051332979635339, 2.9695951874947775, 3.104857793380074, 2.9661093156154057, 2.599444656049039,
                  2.7689099990251993, 3.1079681457019963, 3.2175003308020975, 3.13915131492571, 2.990865162112082,
                  2.809182368445612, 2.8308645660003884, 2.7722355214488394, 2.6906898152131142, 2.712773052333294,
                  2.907598651864276, 3.118564002340879, 2.267783301576401, 1.6557093685779283, 1.944525117993351,
                  2.125672171706464, 2.2354711544027106, 2.5258810418631907, 2.293520619448333, 1.9155510288367914,
                  1.749128579308098, 1.68604534421895, 2.0906649506260977, 2.9464418377140142, 2.280339124282445,
                  1.3992197518807268, 1.077863618717081, 1.0110515783073264, 1.00970279137713, 0.8510017173862998,
                  0.7599038059896258, 0.6482348721791641, 0.5408479054784006, 0.652478305427353, 0.6966139377837801,
                  0.49551013555236306, 0.43235050806323344, 0.4748079325718983, 0.5110994504879418, 0.6078026412654347,
                  0.6525862195053691, 0.5884467738382017, 0.519719003046459, 0.42239293876052225, 0.33889315764332034,
                  0.3459595163400509, 0.37710458372873584, 0.5879050986382746, 0.7649867498764312, 0.7659583429564106,
                  0.781792523391816, 0.6931041800697718, 0.5965770036222144, 0.5033784286145471, 0.5981588734360973,
                  0.5901737941421039, 0.4031981993544842, 0.35303742129879384, 0.31658470850932036, 0.2720700562871399,
                  0.2628939810981532, 0.269740724256241, 0.26663077541063435, 0.2402168822415495, 0.23936720963366223,
                  0.21556640447891126, 0.16265850133938953, 0.14294460392943745, 0.09977407985923897,
                  0.060082801947592857,
                  0.04485945506594633, 0.05507823237007225, 0.053350479284883984, 0.04753493908307488,
                  0.0528095987150577,
                  0.0771863516828505, 0.11136829610687911, 0.11432940037993951, 0.10839054361460528,
                  0.09165551506859543,
                  0.05130273626542604, 0.07661164389518337, 0.13025762491752005, 0.1604615822650354,
                  0.18829727324740494,
                  0.2606772004852437, 0.41840091183265526, 0.5570284088452427, 0.8706557399318441, 1.1920979814620438,
                  1.136848720063587, 1.3181619530388182, 1.767764158175978, 2.1948431385124554, 2.8252994490481327,
                  3.4942248444072725, 3.9730845939333883, 5.091920890436793, 6.330518211373509, 7.057058775556062,
                  7.859762927310861, 8.98734207608358, 9.53487334357498, 9.44124929074591])

    plt.plot(x,y,color='blue')

    support_nodes = calculate_piecewise_constant_approx(x,y)
    output = create_smooth_variance_curve(support_nodes,x,y)

    plt.plot(x,output,color='orange')

    plt.show()

if __name__ == "__main__":
    main()




