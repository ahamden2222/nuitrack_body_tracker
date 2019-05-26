#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('img_proc')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time 
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from body_tracker_msgs.msg import BodyTrackerArray, BodyTracker, Skeleton

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import random 

import ransac
import plane_test as pt

best_plane = np.array([0,0,0,0])
def graph_best_plane(best_plane, points):
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(points[0], points[1], points[2], c='r', marker='o')
	x= np.arange(-2,4,.01)
	y= np.arange(-2,4,.01)
	xx, yy = np.meshgrid(x, y)
	zz = ((-best_plane[0] * xx - best_plane[1] * yy -best_plane[3])/(best_plane[2]))
	ax.plot_surface(xx, yy, zz, alpha=0.2)
	plt.show()



once = True
def pcl_callback(data):
	
	global best_plane

	t= time.time()
	global once
	cloud_points = list(pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True))

	length_sublist = int(.0015*len(cloud_points))
	cloud_points_sublist = random.sample(set(cloud_points), length_sublist)
	best_plane = ransac.ransac(cloud_points_sublist, 15)
	"""points = [[],[],[]]

	for i in range(0, length_sublist):
		points[0].append(round(cloud_points_sublist[i][0], 4))
		points[1].append(round(cloud_points_sublist[i][1], 4))
		points[2].append(round(cloud_points_sublist[i][2], 4))

	if once:
		once = False
		graph_best_plane(best_plane, points)"""
stage_count = 0
cal_points= [[],[],[],[]]
projector_edge_points = []

def remove_outliers(calibration_points):

	for i in range(0, len(cal_points)):
		sum_x = 0
		sum_y = 0
		sum_z = 0
		for j in range(0, len(cal_points[i])):
			sum_x += cal_points[i][j][0]
			sum_y += cal_points[i][j][1]
			sum_z += cal_points[i][j][2]

		avg_x = sum_x/len(cal_points(i))
		avg_y = sum_y/len(cal_points(i))
		avg_z = sum_z/len(cal_points(i))

		projector_edge_points.append([avg_x,avg_y,avg_z])





def update_projector_edge(calibration_points):

	global projector_edge_points




def sklt_callback(data):
	
	global stage_count
	
	x = data.joint_position_left_hand.x
	y = data.joint_position_left_hand.y
	z = data.joint_position_left_hand.z
	points = np.array([x,y,z])
	
	if (pt.distance_to_plane(best_plane, points)) < .05:
		stage_count += 1
		if 0 <= stage_count < 10:
			cal_points[0].append(pt.closest_point_to_plane(best_plane, points))
			#this is stage one
		if 10 <= stage_count < 20:
			cal_points[1].append(pt.closest_point_to_plane(best_plane, points))
			#this is stage two
		if 20 <= stage_count < 30:
			cal_points[2].append(pt.closest_point_to_plane(best_plane, points))			
			#this is stage three
		if 30 <= stage_count < 40:
			cal_points[3].append(pt.closest_point_to_plane(best_plane, points))			
			#this is stage four
		else: 
			#this is the free stage

	#update calculations for edge points	
	update_projector_edge(cal_points)		
	
rospy.init_node('pcl_proc', anonymous=True)
depth_sub = rospy.Subscriber("/camera/depth_cloud",PointCloud2,pcl_callback)
skeleton_sub = rospy.Subscriber("/body_tracker/skeleton", Skeleton, sklt_callback)

rospy.spin()