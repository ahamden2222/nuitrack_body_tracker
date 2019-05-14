#!/usr/bin/env python3

#all library imports
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.animation as animation
import time

import rospy
from body_tracker_msgs.msg import BodyTrackerArray, BodyTracker, Skeleton

is_plot_init = False
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-2,2)
ax.set_ylim(-2,2)
ax.set_zlim(-2,2)

ax.set_xlabel('z axis')
ax.set_ylabel('x axis')
ax.set_zlabel('y axis')


run_count = 0

def plot(data):
    global is_plot_init
    global ax
    global SINK_RUN_TIME 
    global SOURCE_RUN_TIME 
    global VAL_TO_CAPTURE 
    global counter
    global run_count

    if run_count %1 == 0:
        t = time.time()
        ax.scatter(data.joint_position_left_hand.z, data.joint_position_left_hand.x, data.joint_position_left_hand.y, color = 'blue',marker = 'd')
        ax.scatter(data.joint_position_right_hand.z, data.joint_position_right_hand.x, data.joint_position_right_hand.y, color = 'red',marker = 'd')
        ax.view_init(10, 225)
        plt.pause(.000000000001)
        elapsed = time.time() - t

    run_count +=1

if __name__ == '__main__':
    rospy.init_node("listener")
    rospy.Subscriber("/body_tracker/skeleton", Skeleton, plot)
    plt.ion()
    plt.show(block=True)
    #rospy.spin()
    #^^^^^^^^^^^^ the plt.show() serves as the blocking function 
