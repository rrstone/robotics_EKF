#!/usr/bin/env python
import rospy
import time
import numpy as np

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
from ar_track_alvar_msgs.msg import AlvarMarkers
import laser_geometry.laser_geometry as lg
from Kalman import Kalman
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
import math
import matplotlib.pyplot as plt

START_X = 2.25 # Ground truth of robot initial pose, x-coordinate.
START_Y = 0.0  # Ground truth of robot initial pose, y-coordinate.
FIGURE = "pose.png" # File name of pose graph.

class Robot:
	def __init__(self):
		self.depthSub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.depth_callback)
		self.pose = rospy.Subscriber('/pose', PoseStamped, self.coordinate_callback)
		
		self.d1 = None # Distance from robot to landmark 1. 
		self.d2 = None # Distance from robot to landmark 2.

		self.odom_initial_x = None
		self.odom_initial_y = None

		self.linear_velocity = 0
		self.angular_velocity = 0

		self.pose_x = 0
		self.pose_y = 0
		self.angular_displacement = 0

		self.delta_t = 0 # Change in time between pose messages.
		self.delta_d = 0 # Change in distance between pose messages.
		self.delta_angle = 0

		self.current_time = rospy.get_time()

		# Initial State: [x, y, phi, linear velocity]
		self.starting_state = np.array([[START_X, START_Y, 0, 0]]).reshape(4, 1) # Convert to 4x1 vector.

		# Initial velocity: [linear velocity, angular velocity]
		self.control_state = np.array([[0, 0]]).reshape(2, 1)

		self.filter = Kalman(self.starting_state, self.control_state)

		rospy.sleep(2)

	def coordinate_callback(self, message):
		# Track robot pose from odometry at t=0 in the event that initial odom value is non-zero.
		if self.odom_initial_x == None:
			self.odom_initial_x = message.pose.position.x
		if self.odom_initial_y == None:
			self.odom_initial_y = message.pose.position.y

		self.delta_angle = (message.pose.orientation.z - self.angular_displacement)
		self.delta_d = message.pose.position.x - self.odom_initial_x - self.pose_x
		self.delta_t = (rospy.get_time() - self.current_time)

		self.linear_velocity = float(self.delta_d/self.delta_t)
		self.angular_velocity = float(self.delta_angle/self.delta_t)
		self.control_state = np.array([[self.linear_velocity, self.angular_velocity]]).reshape(2, 1)
		self.current_time = rospy.get_time()
		
		self.pose_x = message.pose.position.x - self.odom_initial_x
		self.pose_y = message.pose.position.y - self.odom_initial_y
		self.angular_displacement = message.pose.orientation.z

		# Call Kalman Filter prediction with new odometry values.
		self.filter.prediction(self.control_state, self.angular_displacement, self.delta_t)

	def depth_callback(self, msg):
		self.d1 = None
		self.d2 = None
		# Update depth to landmarks with data from AR tags/RGB-D camera.
		if len(msg.markers) != 0:
			for marker in msg.markers:
				if marker.id == 0: # AR tag 1
					self.d1 = marker.pose.pose.position.x
				if marker.id == 1: # AR tag 2
					self.d2 = marker.pose.pose.position.x

		# If both landmarks are in frame, call Kalman Filter update step with new depth values.
		if self.d1 != None and self.d2 != None:
			 	self.filter.update(self.pose_x, self.pose_y, self.d1, self.d2, self.angular_displacement)

	def plot(self):
		estimated_poses_x = [] # As gathered by the Kalman Filter. 
		estimated_poses_y = []
		poses_x = [] # As gathered by the wheel odometry.
		poses_y = []
		for matrix in self.filter.final_predictions:
			estimated_poses_x.append(matrix[0][0])
			estimated_poses_y.append(matrix[1][0])
		for poses in self.filter.real_poses:
			poses_x.append(poses[0])
			poses_y.append(poses[1])
		x_axis = range(len(poses_x))
		plt.plot(x_axis, estimated_poses_x, label="kalman estimated x-coordinate")
		plt.plot(x_axis, estimated_poses_y, label="kalman estimated y-coordinate")
		plt.plot(x_axis, poses_x, label="odom x-coordinate")
		plt.plot(x_axis, poses_y, label="odom y-coordinate")
		plt.title('Robot X, Y Pose')
		plt.xlabel('Arbitrary Unit of Time')
		plt.ylabel('Pose in Meters')
		plt.savefig(FIGURE)
		plt.legend(loc='center left')
		plt.show()

	def start(self):
		while not rospy.is_shutdown():
			print("Estimated Pose: (" + str(self.filter.x_t[0][0]) + "," + str(self.filter.x_t[1][0]) + ")")
			pass
		# Plot data.
		self.plot()

if __name__ == '__main__':
	rospy.init_node('CONTROLLER')
	r = Robot()
	print("EKF node has been created.")
	r.start()
