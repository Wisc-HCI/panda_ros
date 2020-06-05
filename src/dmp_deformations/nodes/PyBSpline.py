""" Uniform B-spline surfaces
    based on user input of control
    points and curve order

 Created: 05/07/2020
"""

__author__ = "Mike Hagenow"

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import math
import csv

class BSplineSurface:

    def initialize(self,k=3,control_pts=np.zeros((1,1,3))):
        # Parameters for building B-Spline Surface
        self.k = k # order of the B-spline
        self.controls_pts = control_pts # n control points by 3 (x,y,z)
        num_control_pts = np.shape(control_pts)[0:2]
        if num_control_pts[0] <= k or num_control_pts[1]<=k:
            raise Exception("Parameter","Not enough control pts for curve degree")
        self.m = num_control_pts[0]-1 # number of control points in u-direction 0-> m
        self.n = num_control_pts[1]-1  # number of control points in v-direction 0 -> n

        # Need to add the first and last value k times (total of (m-k+2) = ((m+k+2) - 2k)
        self.knots_u = np.concatenate([np.zeros((self.k,)), np.linspace(0, 1, (self.m-self.k+2)),
                                       np.ones((self.k,))])  # number uniform knots in u-direction
        self.knots_v = np.concatenate([np.zeros((self.k,)), np.linspace(0, 1, (self.n-self.k+2)),
                                       np.ones((self.k,))])  # number uniform knots in v-direction

    def calculate_surface_point(self,u,v):
        # Calculate all of the basis functions for the given u,v
        # using recursive formulation
        r = np.array([0.0, 0.0, 0.0])

        ########################################
        # Calculate the interpolated point     #
        ########################################

        # Calculate the sum of basis functions for the point interpolation
        for ii in range(0,self.m+1):
            for jj in range(0,self.n+1):
                r+=self.getN(ii,self.k,u,self.knots_u)*self.getN(jj,self.k,v,self.knots_v)*self.controls_pts[ii,jj,:]

        ########################################
        # Calculate the normal                 #
        ########################################

        # Partial in the U-direction
        r_u = np.array([0.0, 0.0, 0.0])
        for ii in range(0,self.m):
            for jj in range(0,self.n+1):
                scaling_temp =0.0
                if (self.knots_u[ii+self.k+1]-self.knots_u[ii+1]) != 0:
                    scaling_temp = self.k/(self.knots_u[ii+self.k+1]-self.knots_u[ii+1])
                r_u+=self.getN(ii,self.k-1,u,self.knots_u[1:-1])*self.getN(jj,self.k,v,self.knots_v)*scaling_temp*(self.controls_pts[ii+1,jj,:]-self.controls_pts[ii,jj,:])

        # Partial in the V-direction
        r_v = np.array([0.0, 0.0, 0.0])
        for ii in range(0, self.m+1):
            for jj in range(0, self.n):
                scaling_temp = 0.0
                if (self.knots_v[jj + self.k + 1] - self.knots_v[jj + 1]) != 0:
                    scaling_temp = self.k / (self.knots_v[jj + self.k + 1] - self.knots_v[jj + 1])
                r_v += self.getN(ii, self.k, u, self.knots_u) * self.getN(jj, self.k-1, v,self.knots_v[1:-1]) * scaling_temp * (self.controls_pts[ii, jj + 1, :] - self.controls_pts[ii, jj, :])


        # Get the surface normal from the cross-product
        r_u_norm = np.divide(r_u,np.linalg.norm(r_u))
        r_v_norm = np.divide(r_v,np.linalg.norm(r_v))
        n_hat = np.cross(r_u_norm,r_v_norm)

        # return the calculated point
        return r, n_hat, r_u_norm, r_v_norm

    def getN(self,i,p,x,t):
        # Recursive function that calculates the basis
        # function value using De Boor's
        # (simple version of the algorithm with nan checking)

        if p is 0: # 0-th level is a boolean of sorts
            if x>=t[i] and x<t[i+1]: # Firing condition
                return 1.0
            else: # No Fire Condition
                return 0.0

        else: # other levels are recursions based on lower levels
            part_a = 0.0
            part_b = 0.0

            # Check for the two potential divide by zero cases (should be 0)
            if((t[i+p]-t[i]) != 0):
                part_a = (x-t[i])/(t[i+p]-t[i])*self.getN(i,p-1,x,t)
            if (t[i+p+1]-t[i+1]) !=0:
                part_b = (t[i+p+1]-x)/(t[i+p+1]-t[i+1])*self.getN(i+1,p-1,x,t)
            return part_a + part_b

    def writeSurface(self,filename):
        with open('/home/mike/Documents/MikePanda/devel/lib/dmp_deformations/'+filename+'.csv', 'w') as csvfile:
            # Only parameters needed are degree and control points
            csvfile.write(str(self.k) + ',' + str(self.m) + ',' + str(self.n))
            csvfile.write('\n')

            # Load all of the control points
            for ii in range(0, self.m + 1):
                for jj in range(0, self.n + 1):
                    csvfile.write(str(self.controls_pts[ii,jj,0])+' '+str(self.controls_pts[ii,jj,1])+' '+
                                  str(self.controls_pts[ii,jj,2])+',')
                csvfile.write('\n')

    def loadSurface(self, filename):
        with open('/home/mike/Documents/MikePanda/devel/lib/dmp_deformations/' + filename + '.csv') as csvfile:
            surfacereader = csv.reader(csvfile,delimiter=',')

            # Load the parameters and control points
            row = 0
            for row_temp in surfacereader:
                if row==0:
                    k = int(row_temp[0])
                    num_control_u = int(row_temp[1])
                    num_control_v = int(row_temp[2])
                    control_pts = np.zeros((num_control_u+1,num_control_v+1,3))
                else:
                    col=0
                    for ii in range(0,num_control_v+1):
                        point = row_temp[ii]
                        values = point.split(' ')
                        # one extra row for parameters
                        control_pts[row-1,col,0]=float(values[0])
                        control_pts[row-1,col,1]=float(values[1])
                        control_pts[row-1,col,2]=float(values[2])
                        col+=1
                row+=1

            # initialize the B-Spline
            self.initialize(k=k,control_pts=control_pts)



    #### Allow for calculation of an entire path perhaps?

    #### TODO: work with Emmanuel to come up with the requirements

    #### NEED TO ADD SURFACE LEARNING

    #### ADD SURFACE ICP FITTING BASED ON KNOWN MODEL AND POINT CLOUD?

    #### NEED TO ADD CLOSEST SURFACE PROJECTION FOR ANOTHER ARBITRARY 3D POINT



def createCurved():
    x = np.linspace(0.15, -0.35, 25)
    y = np.linspace(0.05, -0.25, 25)
    xv, yv = np.meshgrid(x, y)
    z = 0.865+0.05*np.sin(15*(0.15-xv))+0.06*np.sin((np.pi/0.3)*(0.05-yv)-np.pi/2)

    control_pts = np.transpose([xv, yv, z])
    # control_pts = np.ones((5,5,3))
    print("Control Points:")
    print(np.shape(control_pts))
    print(control_pts[0,24,:])

    # Create a B-Spline instance
    test_curve = BSplineSurface()
    test_curve.initialize(k=3, control_pts=control_pts)

    a,b,c,d = test_curve.calculate_surface_point(0.2, 0.8)
    print("PT:",a)

    # Evaluate a new point
    # eval_pt, n_hat = test_curve.calculate_surface_point(0.5, 0.25)
    # print "NHAT: ", n_hat

    # 3D plotting code
    ax = plt.gca(projection='3d')
    ax.scatter(xv.flatten(), yv.flatten(), z.flatten(), color='blue')
    ax.scatter(a[0],a[1],a[2],color='red')
    # ax.scatter(eval_pt[0], eval_pt[1], eval_pt[2], color='red', s=50)
    # ax.quiver(eval_pt[0], eval_pt[1], eval_pt[2], n_hat[0], n_hat[1], n_hat[2], length=0.1, normalize=True)
    # ax.set_xlim3d(0, 1)
    # ax.set_ylim3d(0, 1)
    # ax.set_zlim3d(0, 1)

    # # 2D plotting code
    # ax = plt.gca()
    # ax.scatter(xv.flatten(), z.flatten(), color='blue')
    # ax.scatter(eval_pt[0], eval_pt[2], color='red', s=50)
    # ax.quiver(eval_pt[0], eval_pt[2],n_hat[0], n_hat[2])

    plt.show()

    test_curve.writeSurface('curved')

def createTable():
    x = np.linspace(0.32, -0.68, 25)
    y = np.linspace(0.18, -0.32, 25)
    xv, yv = np.meshgrid(x, y)
    z = 0.975+0.0*xv+0.0*yv

    control_pts = np.transpose([xv, yv, z])
    # control_pts = np.ones((5,5,3))
    print("Control Points:")
    print(np.shape(control_pts))
    print(control_pts[0,24,:])

    # Create a B-Spline instance
    test_curve = BSplineSurface()
    test_curve.initialize(k=3, control_pts=control_pts)

    a,b,c,d = test_curve.calculate_surface_point(0.3, 0.6)
    print("PT:",a)

    # Evaluate a new point
    # eval_pt, n_hat = test_curve.calculate_surface_point(0.5, 0.25)
    # print "NHAT: ", n_hat

    # 3D plotting code
    ax = plt.gca(projection='3d')
    ax.scatter(xv.flatten(), yv.flatten(), z.flatten(), color='blue')
    ax.scatter(a[0],a[1],a[2],color='red')
    # ax.scatter(eval_pt[0], eval_pt[1], eval_pt[2], color='red', s=50)
    # ax.quiver(eval_pt[0], eval_pt[1], eval_pt[2], n_hat[0], n_hat[1], n_hat[2], length=0.1, normalize=True)
    # ax.set_xlim3d(0, 1)
    # ax.set_ylim3d(0, 1)
    # ax.set_zlim3d(0, 1)

    # # 2D plotting code
    # ax = plt.gca()
    # ax.scatter(xv.flatten(), z.flatten(), color='blue')
    # ax.scatter(eval_pt[0], eval_pt[2], color='red', s=50)
    # ax.quiver(eval_pt[0], eval_pt[2],n_hat[0], n_hat[2])

    plt.show()

    test_curve.writeSurface('table')

def testing():
    # Create surface test data
    x = np.linspace(0, 1, 25)
    y = np.linspace(0, 1, 25)
    xv, yv = np.meshgrid(x, y)
    z = 0.1 * np.abs(np.sin(xv * np.pi*1)) + 0.2 * np.sin(yv * np.pi*2)+0.2

    control_pts = np.transpose([xv, yv, z])
    # control_pts = np.ones((5,5,3))
    print("Control Points:")
    print(np.shape(control_pts))

    # Create a B-Spline instance
    test_curve = BSplineSurface()
    test_curve.initialize(k=3, control_pts=control_pts)

    # Evaluate a new point
    eval_pt, n_hat = test_curve.calculate_surface_point(0.5, 0.25)
    print "NHAT: ",n_hat

    # 3D plotting code
    ax = plt.gca(projection='3d')
    ax.scatter(xv.flatten(), yv.flatten(), z.flatten(), color='blue')
    ax.scatter(eval_pt[0], eval_pt[1], eval_pt[2], color='red', s=50)
    ax.quiver(eval_pt[0], eval_pt[1], eval_pt[2],n_hat[0], n_hat[1], n_hat[2], length=0.1, normalize=True)
    ax.set_xlim3d(0,1)
    ax.set_ylim3d(0,1)
    ax.set_zlim3d(0,1)

    # # 2D plotting code
    # ax = plt.gca()
    # ax.scatter(xv.flatten(), z.flatten(), color='blue')
    # ax.scatter(eval_pt[0], eval_pt[2], color='red', s=50)
    # ax.quiver(eval_pt[0], eval_pt[2],n_hat[0], n_hat[2])

    plt.show()

    test_curve.writeSurface('test')

def unit_testing():
    test_curve = BSplineSurface()
    print(test_curve.getN(0,2,0.25,np.array([0.0, 0.0, 0.0, float(1.0/3), float(2.0/3), 1.0, 1.0, 1.0])))

def curvedLoad():
    test = BSplineSurface()
    test.loadSurface("curved")
    print(test.calculate_surface_point(0.1,0.1))

if __name__ == "__main__":
    testing()




