#!/usr/bin/env python

from __future__ import print_function

import roslib
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import time 
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import random
import plane_test as pt
import ransac


def callback(data):
  array = data.data.astype(int)
  set_rand_points = set()

  #print(array.size)
  #print(array)
  for i in range(0,50):
    rand_row = random.randint(0,479)
    rand_col = random.randint(0,639)
    depth_value = array[rand_row*640 + rand_col]

    set_rand_points.add((rand_row-240,rand_col-320,depth_value))

  print(ransac.ransac(set_rand_points, 50))

def listener():
    rospy.init_node('listener')
    rospy.Subscriber("image_depth", numpy_msg(Floats), callback)
    rospy.spin()

if __name__ == '__main__':
    listener()