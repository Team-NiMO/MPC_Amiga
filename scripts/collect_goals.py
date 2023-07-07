#!/usr/bin/env python 

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
# from move_base_msgs.msg import MoveBaseActionGoal
from rosgraph_msgs.msg import Clock
import numpy as np
import time
from actionlib_msgs.msg import GoalStatusArray
import os

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

#goals = [[-10.72,43.9,0,0,-0.72,0.68],[-10.29,40.15,0,0,0.05,0.99],[-5.24,40.75,0,0,0.05,0.99],[3.03,42.25,0,0,-0.09,0.99]]
next = False
class Goal_reacher:
	def __init__(self):
		self.robot_position = None
		self.curr_time = Clock()
		self.sub = rospy.Subscriber('rover/piksi/position_receiver_0/ros/transform_enu', TransformStamped, self.callback)
		self.sub2 = rospy.Subscriber('/clock', Clock, self.callback2 )
		#self.sub3 = rospy.Subscriber('move_base/status', GoalStatusArray, self.callback3)
	
		# self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
		self.i = 0
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
		self.robot_position = [data.transform.translation.x, data.transform.translation.y ,0,0,data.transform.rotation.x, data.transform.rotation.w]
	def callback2(self, data):
		self.curr_time = data
        
                
	def pub_next(self):
		g = self.i%len(goals)
		goal = goals[g]
		pub_goal = PoseStamped()
		print(goal, gr.robot_position, gr.curr_time)
		pub_goal.header.frame_id = 'odom'
		pub_goal.header.seq = 0
		#pub_goal.header.stamp.secs = gr.curr_time.clock.secs
		#pub_goal.header.stamp.nsecs = gr.curr_time.clock.nsecs
		pub_goal.pose.position.x = goal[0]
		pub_goal.pose.position.y = goal[1]
		pub_goal.pose.orientation.x = goal[2]
		pub_goal.pose.orientation.y = goal[3]
		pub_goal.pose.orientation.z = goal[4]
		pub_goal.pose.orientation.w = goal[5]
		for j in range(1):
			time.sleep(0.5)
			self.pub.publish(pub_goal)
		#print('published: ',self.i, '   ', pub_goal)
		x = rospy.get_rostime()
		
		self.i += 1
		

if __name__ == "__main__":
	
	gr = Goal_reacher()
	rospy.init_node('goal_reacher', anonymous=True)
	while True:
            x = input('Save_point?')
            if x=='y':
               c = np.loadtxt('test.csv',delimiter=',')
               print(c)
               if(len(c) == 0):
                 print(gr.robot_position)
                 np.savetxt('test.csv',[gr.robot_position],delimiter=',')
               else:
                 print('position ',gr.robot_position)
                 print('here')
                 np.savetxt('test.csv',np.vstack([c,gr.robot_position]),delimiter=',')

	print(rospy.get_time())
	rospy.spin()
