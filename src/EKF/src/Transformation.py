import math
import numpy as np
"""
Utility file which handles tranformations between different frames of reference
"""

START_X = 2.25 # Robot's x distance away from the landmark
START_Y = 0.0 # Robot's y distance away from the landmark

def Pose_to_Landmark(dx, dy, theta):
	T_o_l = [[1, 0, START_X],
			 [0, 1, START_Y],
			 [0, 0, 1]] # Transform from odom to the landmark.
	T_r_o = [[math.cos(theta), -1.0 * math.sin(theta), -1.0 * dx],
			 [math.sin(theta), math.cos(theta), -1.0 * dy],
			 [0, 0, 1]] # Transform from the robot to the odom.

	T = np.dot(T_r_o, T_o_l)

	return [T[0][2], T[1][2]]