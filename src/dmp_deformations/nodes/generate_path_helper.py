#!/usr/bin/env python
import numpy as np

def printPathSection(csvfile,starting_point,ending_point,num_pts):
    for ii in range(0,num_pts):
        interp_x = starting_point[0]+(ending_point[0]-starting_point[0])*(float(ii)/float(num_pts-1))
        interp_y = starting_point[1]+(ending_point[1]-starting_point[1])*(float(ii)/float(num_pts-1))
        interp_z = starting_point[2]+(ending_point[2]-starting_point[2])*(float(ii)/float(num_pts-1))
        interp_fx = starting_point[3]+(ending_point[3]-starting_point[3])*(float(ii)/float(num_pts-1))
        interp_fy = starting_point[4]+(ending_point[4]-starting_point[4])*(float(ii)/float(num_pts-1))
        interp_fz = starting_point[5]+(ending_point[5]-starting_point[5])*(float(ii)/float(num_pts-1))
       
        csvfile.write(str(interp_x)+','+str(interp_y)+','+str(interp_z)+','+str(interp_fx)+','+str(interp_fy)+','+str(interp_fz))
        csvfile.write('\n')

def main():
    num_pts = 100

    # with open('/home/mike/Documents/MikePanda/devel/lib/dmp_deformations/sanding.csv', 'w') as csvfile:
    with open('/home/mike/Documents/MikePanda/devel/lib/dmp_deformations/writing.csv', 'w') as csvfile:
    # with open('/home/mike/Documents/MikePanda/devel/lib/dmp_deformations/painting.csv', 'w') as csvfile:

        # csvfile.write("0,100,200")
        # csvfile.write('\n')
        # csvfile.write("1,0,1")
        # csvfile.write('\n')
        # printPathSection(csvfile,np.array([0.16, 0.05, 1.3, 0.0, 0.0, -5.0]),np.array([0.16, 0.05, 0.90, 0.0, 0.0, -5.0]),num_pts)
        # printPathSection(csvfile,np.array([0.16, 0.05, 0.9, 0.0, 0.0, -5.0]),np.array([0.0, 0.05, 0.9, 0.0, 0.0, -5.0]),num_pts)
        # printPathSection(csvfile,np.array([0.0, 0.05, 0.9, 0.0, 0.0, -5.0]),np.array([0.0, 0.05, 1.3, 0.0, 0.0, -5.0]),num_pts)

        csvfile.write("0,100,200,300,400")
        csvfile.write('\n')
        csvfile.write("1,0,0,0,1")
        csvfile.write('\n')
        printPathSection(csvfile,np.array([-0.6, 0.078, 1.2, 0.0, 0.0, -5.0]),np.array([-0.6, 0.078, 1.00, 0.0, 0.0, -5.0]),num_pts)
        printPathSection(csvfile,np.array([-0.6, 0.078, 1.00, 0.0, 0.0, -5.0]),np.array([0.195, 0.088, 1.00, 0.0, 0.0, -5.0]),num_pts)
        printPathSection(csvfile,np.array([0.195, 0.088, 1.00, 0.0, 0.0, -5.0]),np.array([0.195, -0.165, 1.00, 0.0, 0.0, -5.0]),num_pts)
        printPathSection(csvfile,np.array([0.195, -0.165, 1.00, 0.0, 0.0, -5.0]),np.array([-0.6, -0.165, 1.00, 0.0, 0.0, -5.0]),num_pts)
        printPathSection(csvfile,np.array([-0.6, -0.165, 1.00, 0.0, 0.0, -5.0]),np.array([-0.6, -0.165, 1.2, 0.0, 0.0, -5.0]),num_pts)

        # csvfile.write("0,100,200,300,400")
        # csvfile.write('\n')
        # csvfile.write("1,1,1,1,1")
        # csvfile.write('\n')
        # printPathSection(csvfile,np.array([0.2, 0.1, 1.1, 0.0, 0.0, 0.0]),np.array([-0.5, 0.1, 1.1, 0.0, 0.0, 0.0]),num_pts)
        # printPathSection(csvfile,np.array([-0.5, 0.1, 1.1, 0.0, 0.0, 0.0]),np.array([-0.5, -0.05, 1.1, 0.0, 0.0, 0.0]),num_pts)
        # printPathSection(csvfile,np.array([-0.5, -0.05, 1.1, 0.0, 0.0, 0.0]),np.array([0.2, -0.05, 1.1, 0.0, 0.0, 0.0]),num_pts)
        # printPathSection(csvfile,np.array([0.2, -0.05, 1.1, 0.0, 0.0, 0.0]),np.array([0.2, -0.2, 1.1, 0.0, 0.0, 0.0]),num_pts)
        # printPathSection(csvfile,np.array([0.2, -0.2, 1.1, 0.0, 0.0, 0.0]),np.array([-0.5, -0.2, 1.1, 0.0, 0.0, 0.0]),num_pts)
        
        


if __name__ == "__main__":
    main()




