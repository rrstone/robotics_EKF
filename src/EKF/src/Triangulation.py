import math
import numpy as np

"""
Utility file which handles information about observation and localization.
"""

# Returns the gamma angle of a triangle with points L1-L2-R.
# length 1, length 2 = distance between R and L1, L2 respectively.
# delta = distance between points L1 and L2.
def gamma_angle(length_1, length_2, delta):
	numerator = (length_2 ** 2) - (delta ** 2) - (length_1 ** 2)
	denom = -2.0 * length_1 * length_2
	return math.acos(numerator/denom)

# Function for an pose observation givrn depth readings to landmarks 1 and 2 and gamma angle.
def z_t(depth_1, depth_2, gamma):
	return np.array([[depth_1 * math.sin(gamma), depth_2 * math.cos(gamma)]])