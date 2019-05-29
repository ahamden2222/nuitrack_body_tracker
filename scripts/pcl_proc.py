#!/usr/bin/env python
from __future__ import print_function

import roslib
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

import math

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

best_plane = np.array([0,0,0,0])
is_best_plane_found = True

SINK_RUN_TIME = 30
SOURCE_RUN_TIME = 30
VAL_TO_CAPTURE = 1
counter = 0

def callback_optimizer(data):
	global SINK_RUN_TIME
	global SOURCE_RUN_TIME
	global VAL_TO_CAPTURE
	global counter

	VAL_TO_CAPTURE = (1./SINK_RUN_TIME) / (1./SOURCE_RUN_TIME)
	VAL_TO_CAPTURE *= 1.05
	VAL_TO_CAPTURE = int(math.ceil(VAL_TO_CAPTURE))
	
	counter += 1

	if counter >= VAL_TO_CAPTURE:
		counter = 0
		t = time.time()
		skeleton_callback(data)
		elapsed = time.time() - t
		print(elapsed)
		SINK_RUN_TIME = 1/elapsed

def graph_best_plane(best_plane, points):
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(points[0], points[1], points[2], c='r', marker='o')
	x= np.arange(-2,4,1)
	y= np.arange(-2,4,1)
	xx, yy = np.meshgrid(x, y)
	zz = ((-best_plane[0] * xx - best_plane[1] * yy -best_plane[3])/(best_plane[2]))
	#ax.plot_surface(xx, yy, zz, alpha=0.2)
	plt.show()

"""
fig = plt.figure()
plt.xlim(0,854)
plt.ylim(0, 480)
plt.axis('off')
plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
curve = plt.scatter(0,0)"""
def plot_pixel(pixel_x, pixel_y):
	global curve
 	pixel_x = int(pixel_x)
	pixel_y = int(pixel_y)
	curve = plt.scatter(pixel_x,pixel_y)
	plt.show()

def pcl_callback(data):
	
	global best_plane
	global is_best_plane_found
	
	if  is_best_plane_found:
		cloud_points = list(pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True))

		length_sublist = int(.0015*len(cloud_points))
		cloud_points_sublist = random.sample(set(cloud_points), length_sublist)
		best_plane = ransac.ransac(cloud_points_sublist, 15)
		"""
		points = [[],[],[]]
		for i in range(0, length_sublist):
			points[0].append(round(cloud_points_sublist[i][0], 4))
			points[1].append(round(cloud_points_sublist[i][1], 4))
			points[2].append(round(cloud_points_sublist[i][2], 4))

		graph_best_plane(best_plane, points)
		is_best_plane_found = False"""


def skeleton_callback(data):

	x = data.joint_position_left_hand.x
	y = data.joint_position_left_hand.y
	z = data.joint_position_left_hand.z
	point = np.array([x,y,z])

	
	if (pt.distance_to_plane(best_plane, point)) < .3:
		print("contact")
		
		closest_point = pt.closest_point_to_plane(best_plane, point)
		horizontal = closest_point[0]
		vertical = closest_point[2]
		horizontal = ((horizontal+1.1)/2.20)*1920
		vertical = (vertical/1.20)*1080
		touch_point = np.array([horizontal, vertical], dtype=np.float32)
		point_pub.publish(touch_point)

rospy.init_node('pcl_proc', anonymous=True)
depth_sub = rospy.Subscriber("/camera/depth_cloud",PointCloud2,pcl_callback)
skeleton_sub = rospy.Subscriber("/body_tracker/skeleton", Skeleton, callback_optimizer)
point_pub = rospy.Publisher("touch_points", numpy_msg(Floats), queue_size=100)
rospy.spin()
