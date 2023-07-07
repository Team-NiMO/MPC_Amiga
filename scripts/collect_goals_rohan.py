#!/usr/bin/env python 

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
# from sensor_msgs.msg import Imu
# from move_base_msgs.msg import MoveBaseActionGoal
from rosgraph_msgs.msg import Clock
import numpy as np
import time
from actionlib_msgs.msg import GoalStatusArray
import os
from tf.transformations import euler_from_quaternion

#goals = [[-4.84,5.17,0,0,0.71,0.70], [4.97,4.84,0,0,0,-0.69,0.71],[5.2,-4.84,0,0,-0.69,0.71],[-4.64,-4.75,0,0,-0.99,0.00]]

#goals = [[-00, -35.0, 0, 0, 0.54, 0.84], [25.0, -5.0, 0, 0, -0.65, 0.76], [25.0, -35.0, 0, 0, -0.99, 0.01], [-0.0,-5.07,0,0,0.04,0.99]]

#goals = [[-00, -35.0, 0, 0, 0.0, 1],[5, -35, 0, 0,  0.59, 0.8], [9.3, -1,0,0,-0.07, 0.99],[23,-3,0,0,-0.77,0.63],[10,35,0,0,0.54, 0.84]]
goals = [[8.9, 13, 0, 0, -0.3, 0.95],[52, -8.7, 0, 0, -0.89, 0.45], [41,-28,0,0,0.97, 0.20],[-4,-12,0,0,0.46,0.88]]

goals = [
[-13.98,44.89,0,0,-0.74,0.66],
[-13.32,39.6,0,0,0.11,0.99],
[-4.6,40.9,0,0,0.11,0.99],
[2.6,42.2,0,0,0.08,0.99],
[11.2,43.9,0,0,0.08,0.99],
[18.2,45.2,0,0,0.08,0.99],
[25.3,46.5,0,0,0.08,0.99],
[34.3,48.4,0,0,0.08,0.99],
[43.9,50.3,0,0,0.08,0.99],
[53.7,52.3,0,0,0.08,0.99],
[63.8,54.3,0,0,0.08,0.99],
[71.7,56.3,0,0,0.71,0.69],
[70.9,60.9,0,0,-0.99,0.03],
[65.2,59.2,0,0,-0.99,0.03],
[55.8,57.8,0,0,-0.99,0.03],
[47.4,56.04,0,0,-0.99,0.03],
[37.8,54.2,0,0,-0.99,0.03],
[22.56,51.2,0,0,-0.99,0.03],
[9.9,48.9,0,0,-0.99,0.02],
[-1.77, 46.72,0,0,-0.99,0.07]
]

next = False
class Goal_reacher:
	def __init__(self):
		self.robot_position_odom = []
		self.robot_position_gps = []
		self.curr_time = Clock()
		# self.sub = rospy.Subscriber('/odometry/filtered', Odometry, self.callback)
		self.sub1 = rospy.Subscriber('rover/piksi/position_receiver_0/ros/transform_enu', TransformStamped, self.callback1)
		# self.sub2 = rospy.Subscriber('/clock', Clock, self.callback2 )
		# self.sub3 = rospy.Subscriber('/imu/data', Imu, self.callback_yaw)
		#self.sub3 = rospy.Subscriber('move_base/status', GoalStatusArray, self.callback3)
	
		self.stamp_old = 0

	def callback3(self, data):
		
		m = data.status_list[-1]
		stamp = data.header.stamp.secs
		if m.text == "Goal reached." and stamp-self.stamp_old>1.0:
			print('STATUS : ', m.status)
			self.pub_next()
			print(data.header)
			self.stamp_old = stamp
			
	def callback(self, data):
		self.robot_position_odom.append([data.transform.translation.x, data.transform.translation.y, 0., 0., 0., 0.])
	
	def callback1(self, data):
		robot_orientation = data.transform.rotation
		r_quaternion_list = [robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w]
		_, _, yaw = euler_from_quaternion(r_quaternion_list)
		self.robot_position_gps.append([data.transform.translation.x, data.transform.translation.y, yaw, 0., 0., 0.])
	
	def callback2(self, data):
		self.curr_time = data

if __name__ == "__main__":
	
	# f1 = 'data/2023-05-26/2023-05-26_flagstaff_oval_ekf.txt'
	f2 = '/home/amiga/navigation_ws/src/mpc_controller/gps_coordinates/rows_real.txt'

	mode = 'subsample'
	subsample_rate = 20

	gr = Goal_reacher()
	rospy.init_node('goal_reacher', anonymous=True)
	if mode == 'capture':
		while True:
			x = input('Save_point?')	
			if x == 'y':
				c1 = np.loadtxt(f1,delimiter=',')
				if(len(c1) == 0):
					np.savetxt(f1,[gr.robot_position_odom[-1]],delimiter=',')
				else:
					np.savetxt(f1,np.vstack([c1,gr.robot_position_odom[-1]]),delimiter=',')

				print(c1)
	else:
		while True:
			x = input('Done?: ')
			if x == 'y':
				break

		# ekf = np.stack(gr.robot_position_odom)[::subsample_rate]
		gps = np.stack(gr.robot_position_gps)[::subsample_rate]
		# np.savetxt(f1, ekf, delimiter=',')
		np.savetxt(f2, gps, delimiter=',')
		print('Saved subsampled trajectories')
	
	rospy.spin()
