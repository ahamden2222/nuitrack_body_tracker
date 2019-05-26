import numpy as np
import random 
import plane_test as pt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

#reference: http://www.cse.psu.edu/~rtc12/CSE486/lecture15.pdf
#reference: http://www.ipb.uni-bonn.de/pdfs/Yang2010Plane.pdf
def ransac(list_of_points, iterations):

	best_plane = np.array([0,0,0,0])
	max_points_captured = 0
	
	for i in range (0, iterations):

		points_captured = 0

		three_point_list = np.array(random.sample(set(list_of_points),3))

		point_1 = three_point_list[0]
		point_2 = three_point_list[1]
		point_3 = three_point_list[2]

		plane = pt.plane_finder(point_1, point_2, point_3)
			
		for p in range(0, len(list_of_points)):
			if pt.distance_to_plane(plane, list_of_points[p] ) < .05:
				points_captured += 1
			
		if points_captured > max_points_captured:
			max_points_captured = points_captured
			best_plane = plane

	return best_plane
