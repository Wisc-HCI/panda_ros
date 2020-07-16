#!/usr/bin/env python

""" Convert STL of desired surface to a BSplineSurface
 using the BSplinesurface calls from the other library
 Created: 07/10/2020
"""

__author__ = "Mike Hagenow"

import trimesh
import trimesh.sample as sample
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize, Bounds
from scipy.spatial import ConvexHull

def exp_map_np(w):
    w = np.array(w)
    w = w.reshape(3,)
    theta = (w[0] ** 2 + w[1] ** 2 + w[2] ** 2) ** 0.5 + 1e-30
    w = w / theta
    w_hat = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
    return np.eye(3) + w_hat * np.sin(theta) + np.dot(w_hat, w_hat) * (1 - np.cos(theta))

# x are the parameters, stored as wx,wy,d
# y are the points in the point cloud
def obj_plane_dist(x, pts):
    sum = 0.0
    for pt in pts:
        # equation for the plane - sum of residual is used as the LSQ error
        vec_to_plane_center = np.transpose(np.matmul(exp_map_np([x[0], x[1], 0.0]),np.array([0.0, 0.0, x[2]]).reshape((3,1)))-pt.reshape((3,1)))
        sum+=np.abs(np.matmul(vec_to_plane_center,np.matmul(exp_map_np([x[0], x[1], 0.0]),np.array([0.0, 0.0, 1.0]).reshape((3,1))))[0][0])
    # print x[0], x[1], x[2], sum
    return sum

def findPlane(pts):
    wx = 0.0
    wy = 0.0
    d =  0.0

    # TODO: add random restarts for local minima...

    x0 = np.array([wx, wy, d])
    bounds = Bounds([0.0, 0.0, -np.inf], [np.pi, np.pi, np.inf])
    res = minimize(obj_plane_dist, x0, method='Nelder-Mead',
                   options={'disp': False}, bounds=None, args=(pts))
    return res.x[0], res.x[1], res.x[2]

def main():
    stl_file = '/home/mike/Desktop/NASAULIlayupFLANGEsurface.STL'
    mesh = trimesh.load(stl_file)

    # Turn STL into a point cloud of the desired surface using even sampling
    points, something = sample.sample_surface_even(mesh,400)

    # convert properly for mm to m issue
    points = points/1000

    ###########################################################################################
    ##  Algorithm for fitting the B Spline Surface to the Point Cloud                        ##
    ##  partially adopted from https://www.inf.usi.ch/hormann/papers/Greiner.1997.IAA.pdf    ##
    ###########################################################################################

    # Note: may also need segmentation, but not in this case

    # Fit a plane to the points using NLSQ
    wx,wy,d = findPlane(points)
    print "OPT:", wx, wy, d

    # Project all of the points onto the plane
    projected_pts = []
    plane_origin = np.matmul(exp_map_np([wx, wy, 0.0]),np.array([0.0, 0.0, d]))
    plane_normal = np.matmul(exp_map_np([wx, wy, 0.0]),np.array([0.0, 0.0, 1.0]))
    for point in points:
        v = point - plane_origin
        dist = np.dot(v,plane_normal)
        proj_point = point - dist*plane_normal

        # convert back to plane parameterized representation -> aka divide by exponential map?
        param_pt = np.matmul(np.transpose(exp_map_np([wx, wy, 0.0])),proj_point.reshape((3,)))
        u_temp = param_pt[0]
        v_temp = param_pt[1]
        projected_pts.append([u_temp, v_temp])
    projected_pts = np.array(projected_pts)

    #######################################################
    # Fit the best rectangle around the points            #
    #######################################################
    # use something like this... https://gis.stackexchange.com/questions/22895/finding-minimum-area-rectangle-for-given-points
    fig = plt.figure()
    plt.scatter(projected_pts[:, 0], projected_pts[:, 1])
    hull = ConvexHull(projected_pts)
    for simplex in hull.simplices:
        plt.plot(projected_pts[simplex, 0], projected_pts[simplex, 1], 'k-', color='red')

    # key idea: orientation of minimized rectangle is the same as one of the edges of the point cloud convex hull


    # create evenly spaced candidates

    # conform back to the surface as control points (Do I need a spring)


    # print points
    # print(np.shape(points))

    fig = plt.figure()
    ax = fig.gca(projection='3d')


    # PLOT THE PLANE!!!
    plane_pts = []
    for ii in np.linspace(-0.2, 0.2, 10):
        for jj in np.linspace(-0.2, 0.2, 10):
            plane_pts.append(np.matmul(exp_map_np([wx, wy, 0.0]),np.array([[ii], [jj], [d]])))
    plane_pts = np.array(plane_pts)

    center_pt = np.matmul(exp_map_np([wx, wy, 0.0]), np.array([[0.0], [0.0], [d]]))
    ax.scatter(center_pt[0], center_pt[1], center_pt[2], color='red')
    ax.scatter(plane_pts[:,0],plane_pts[:,1],plane_pts[:,2],color='orange')
    ax.scatter(points[:,0],points[:,1],points[:,2])
    ax.set_xlim3d(-0.2, 0.2)
    ax.set_ylim3d(-0.2, 0.2)
    ax.set_zlim3d(-0.2, 0.2)

    plt.show()

if __name__ == "__main__":
    main()




