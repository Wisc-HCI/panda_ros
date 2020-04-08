#!/usr/bin/env python
import numpy as np

def printPathSection(csvfile,starting_point,ending_point,num_pts):
    for ii in range(0,num_pts):
        interp_x = starting_point[0]+(ending_point[0]-starting_point[0])*(float(ii)/float(num_pts-1))
        interp_y = starting_point[1]+(ending_point[1]-starting_point[1])*(float(ii)/float(num_pts-1))
        interp_z = starting_point[2]+(ending_point[2]-starting_point[2])*(float(ii)/float(num_pts-1))
       
        csvfile.write(str(interp_x)+','+str(interp_y)+','+str(interp_z)+",0.0,0.0,0.0")
        csvfile.write('\n')

def main():
    starting_point = np.array([0.2, 0.1, 1.1])
    ending_point = np.array([0.5, 0.1, 1.1])

    num_pts = 100

    with open('/home/mike/Documents/MikeKinova/devel/lib/dmp_deformations/painting.csv', 'w') as csvfile:

        csvfile.write("0,100,200,300,400")
        csvfile.write('\n')
        printPathSection(csvfile,np.array([0.2, 0.1, 1.1]),np.array([-0.5, 0.1, 1.1]),num_pts)
        printPathSection(csvfile,np.array([-0.5, 0.1, 1.1]),np.array([-0.5, -0.05, 1.1]),num_pts)
        printPathSection(csvfile,np.array([-0.5, -0.05, 1.1]),np.array([0.2, -0.05, 1.1]),num_pts)
        printPathSection(csvfile,np.array([0.2, -0.05, 1.1]),np.array([0.2, -0.2, 1.1]),num_pts)
        printPathSection(csvfile,np.array([0.2, -0.2, 1.1]),np.array([-0.5, -0.2, 1.1]),num_pts)


if __name__ == "__main__":
    main()




