#!/usr/bin/env python

""" Calculates DMP using LWR
 Created: 02/19/2020
"""

__author__ = "Mike Hagenow"

import matplotlib.pyplot as plt
import numpy as np
from numpy import genfromtxt
import mayavi.mlab as mlab
import os
import rospy
from DMP import DMP
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker

demonstrations = []

pathDMP = None
forceDMP = None
path0 = None
path1 = None
path2 = None
path3 = None
path4 = None

def plot_paths_rviz(demonstration_data_trimmed,trajectories):
    global path0, path1, path2, path3, path4, pathDMP, forceDMP
    # Clear all existing in RVIZ
    emptyPath = Path()
    emptyPath.header.frame_id = 'base_link'
    emptyPath.header.stamp = rospy.Time.now()
    pathDMP.publish(emptyPath)
    path0.publish(emptyPath)
    path1.publish(emptyPath)
    path2.publish(emptyPath)
    path3.publish(emptyPath)
    path4.publish(emptyPath)
    emptyForces = MarkerArray()
    forceDMP.publish(emptyForces)

    for ii in range(0,len(demonstration_data_trimmed)):
        demo = demonstration_data_trimmed[ii]
        tempPath = Path()
        tempPath.header.frame_id = 'base_link'
        tempPath.header.stamp = rospy.Time.now()
        for jj in range(0,np.shape(demo)[0]):
            tempPose = PoseStamped()
            tempPose.header.stamp = rospy.Time.now()
            tempPose.pose.position.x = demo[jj,0]
            tempPose.pose.position.y = demo[jj,1]
            tempPose.pose.position.z = 0.0
            tempPose.pose.orientation.x = 0.0
            tempPose.pose.orientation.y = 0.0
            tempPose.pose.orientation.z = 0.0
            tempPose.pose.orientation.w = 1.0
            tempPath.poses.append(tempPose)
    
        # Publish the original paths
        if ii==0:
            path0.publish(tempPath)
        elif ii==1:
            path1.publish(tempPath)
        elif ii==2:
            path2.publish(tempPath)
        elif ii==3:
            path3.publish(tempPath)
        else:
            # If more than 5 paths, just update last
            path4.publish(tempPath)

        # Publish the DMP
        tempPath = Path()
        tempPath.header.frame_id = 'base_link'
        tempPath.header.stamp = rospy.Time.now()

        tempForces = MarkerArray()
        for jj in range(0,len(trajectories[0])):
            tempPose = PoseStamped()
            tempPose.header.stamp = rospy.Time.now()
            tempPose.pose.position.x = trajectories[0][jj]
            tempPose.pose.position.y = trajectories[1][jj]
            tempPose.pose.position.z = 0.0
            tempPose.pose.orientation.x = 0.0
            tempPose.pose.orientation.y = 0.0
            tempPose.pose.orientation.z = 0.0
            tempPose.pose.orientation.w = 1.0
            tempPath.poses.append(tempPose)

            tempForce = Marker()
            tempForce.color.a = 1.0
            tempForce.color.r = 1.0
            tempForce.color.b = 0.0
            tempForce.color.g = 0.0
            
            if jj % 10 == 0:
                tempForce.pose.position.x = trajectories[0][jj]
                tempForce.pose.position.y = trajectories[1][jj]
                tempForce.pose.position.z = 0.0
                tempForce.pose.orientation.x = 0.0
                tempForce.pose.orientation.y = 0.7068
                tempForce.pose.orientation.z = 0.0
                tempForce.pose.orientation.w = 0.7074
                tempForce.header.frame_id = "base_link"
                tempForce.header.stamp = rospy.Time.now()
                tempForce.id = jj
                tempForce.type = Marker.ARROW
                tempForce.action = Marker.ADD
                tempForce.scale.x = -trajectories[2][jj]/300
                tempForce.scale.y = 0.003
                tempForce.scale.z = 0.003
                tempForces.markers.append(tempForce)


        forceDMP.publish(tempForces)
        pathDMP.publish(tempPath)
        print "Pushing Data to RVIZ" 


def resetDMP(data):
    global demonstrations
    demonstrations = []
    global path0, path1, path2, path3, path4, pathDMP, forceDMP
    # Clear all existing in RVIZ
    emptyPath = Path()
    emptyPath.header.frame_id = 'base_link'
    emptyPath.header.stamp = rospy.Time.now()
    pathDMP.publish(emptyPath)
    path0.publish(emptyPath)
    path1.publish(emptyPath)
    path2.publish(emptyPath)
    path3.publish(emptyPath)
    path4.publish(emptyPath)
    emptyForces = MarkerArray()
    emptyMarker = Marker()
    emptyMarker.action = Marker.DELETEALL
    emptyForces.markers.append(emptyMarker)
    forceDMP.publish(emptyForces)


def loadData(data):
    global demonstrations
    # File structure is a list of arrays (one per demonstration)
    # Each array is an n-samples by 6 array for the 3 positions and 3 forces
    print("Loading Data from CSV")
    directory = '/home/hcilab/Documents/MikePanda/devel/lib/dmp_deformations'
    
    demonstrations.append(data.data)
    demonstration_data = []

    for filename in os.listdir(directory):
        if filename.endswith(".csv") and filename in demonstrations:
            print("Loading File: " + filename)
            demonstration_data.append(np.loadtxt(open(directory + '/' + filename, "rb"), delimiter=",", skiprows=0)[0:-1:20])

    calculateDMP(demonstration_data)


def calculateDMP(demonstration_data):
    print "Building DMP"
    dmp = DMP()

    # Sending x, y, and Fz
    demonstration_data_trimmed = []

    for demo in demonstration_data:
        temp = np.zeros((len(demo),3))
        temp[:,0:2] = demo[0:len(demo),0:2].reshape((len(demo),2))
        temp[:,2] = demo[0:len(demo),5].reshape((len(demo),))

        demonstration_data_trimmed.append(temp)

    dmp.inputData(demonstration_data=demonstration_data_trimmed)
    dmp.computeDMP()

    trajectories = dmp.getTrajectory()

    print "MIN Force DMP: " , np.min(trajectories[2]) , " (N)"
    print "MAX Force DMP: " , np.max(trajectories[2]) , " (N)"

    plot_paths_rviz(demonstration_data_trimmed,trajectories)
    

def main():
    global path0, path1, path2, path3, path4, pathDMP, forceDMP
    rospy.init_node('dmppathlistener', anonymous=True)
    pathDMP = rospy.Publisher('/dmp/pathdmp', Path, queue_size=1)
    forceDMP = rospy.Publisher('/dmp/forcedmp', MarkerArray, queue_size=1)
    path0 = rospy.Publisher('/dmp/path0', Path, queue_size=1)
    path1 = rospy.Publisher('/dmp/path1', Path, queue_size=1)
    path2 = rospy.Publisher('/dmp/path2', Path, queue_size=1)
    path3 = rospy.Publisher('/dmp/path3', Path, queue_size=1)
    path4 = rospy.Publisher('/dmp/path4', Path, queue_size=1)
    rospy.Subscriber("/dmp/filepub", String, loadData)
    rospy.Subscriber("/dmp/reset", String, resetDMP)
    
    rospy.spin()
    # loadData(demonstration_data)
    # calculateDMP(demonstration_data)


if __name__ == "__main__":
    main()




