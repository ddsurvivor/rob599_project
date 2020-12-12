#!/usr/bin/env python

import rospy
import actionlib
import sys
import rospkg

from tf import transformations
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from geometry_msgs.msg import Quaternion

from rob599_project.msg import PickAction, PickGoal, PickFeedback, PickResult
from rob599_project.msg import CatchAction, CatchGoal, CatchFeedback, CatchResult


# navigate robot to the position to catch the goods
# read goods position from files
# make an action server to do ReachGoods, Catch, CarryToHouse

class Robot:
	#pose_dict = []
	def __init__(self):
		self.pkg_dir = rospkg.RosPack().get_path('rob599_project')

		self.active_callback = None
		self.feedback_callback = None
		self.done_callback = None
		self.index = 10

		# Make an action client, and wait for the server.
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.client.wait_for_server()

		# Make a goal template, and fill in all of the relevant fields.
		self.goal = MoveBaseGoal()
		self.goal.target_pose.header.frame_id = 'map'
		self.goal.target_pose.header.stamp = rospy.Time()

		# Pick goods action
		self.server = actionlib.SimpleActionServer('pick_action', PickAction, self.pick_callback, False)
		self.server.start()
		rospy.loginfo('Pick action started')

		# Catch and drop action
		self.catch_server = actionlib.SimpleActionServer('catch_action', CatchAction, self.catch_callback, False)
		self.catch_server.start()
		rospy.loginfo('Catch action started')

		self.catch_client = actionlib.SimpleActionClient('catch_action', CatchAction)

		# publish goods markers
		self.publisher = rospy.Publisher('goods_box_array', MarkerArray, queue_size = 10)

		amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)


		self.house_pose = self.load_data('house_pose.txt')
		self.goods_pose = self.load_data('goods_pose.txt')
		self.house_point = self.load_data('house_point.txt')

		self.set_goods_markers()

		

	# Load goods position from file
	def load_data(self, filename):
		path = self.pkg_dir + '/files/'+ filename
		f = open(path, 'r')
		data = eval(f.read())
		f.close()
		print('load data finished')	
		return data

	# Drive robot to position
	def go_to(self, wait = False):
		# Make the action call.
		self.client.send_goal(self.goal, active_cb=self.active_callback, feedback_cb=self.feedback_callback,
			done_cb=self.done_callback)

		# Wait, if asked to.
		if wait:
			self.client.wait_for_result()

	# set target position
	def find_goal(self, index, target):
		if target == 'goods':
			_x = self.goods_pose[index][0]-0.5
			_y = self.goods_pose[index][1]
			_theta = 0
			self.set_goal(_x, _y, _theta)
		elif target == 'house':
			_x = self.house_pose[index][0]-0.5
			_y = self.house_pose[index][1]
			_theta = 0
			self.set_goal(_x, _y, _theta)

		print('find goal finished')

	def set_goal(self, x, y, theta):
		"""
		:param x: x coordinate in the map frame.
		:param y: y coordinate in the map frame.
		:param theta: heading (angle with the x-axis in the map frame)
		"""

		rospy.loginfo('{0}: Drive robot to pose ({1}, {2}, {3}) '.format(self.__class__.__name__,
			x, y, theta))

		# Set the x and y positions
		self.goal.target_pose.pose.position.x = x
		self.goal.target_pose.pose.position.y = y

		# Set the orientation.
		self.goal.target_pose.pose.orientation = Quaternion(*transformations.quaternion_from_euler(0.0, 0.0, theta))


	def catch(self):
		goal = CatchGoal()
		goal.command = 0
		self.catch_client.send_goal(goal, active_cb=self.active_callback, feedback_cb=self.feedback_callback,
			done_cb=self.done_callback)
		self.catch_client.wait_for_result()
		print('catch finished')

	def drop(self):
		goal = CatchGoal()
		goal.command = 1
		self.catch_client.send_goal(goal, active_cb=self.active_callback, feedback_cb=self.feedback_callback,
			done_cb=self.done_callback)
		self.catch_client.wait_for_result()
		print('drop finished')

	def robot_pick(self, index):
		rospy.loginfo('Pick {0}'.format(index))
		self.find_goal(index, 'goods')
		self.go_to(wait = True)
		self.catch()
		self.find_goal(index, 'house')
		self.go_to(wait = True)
		self.drop()

	def pick_callback(self, goal):
		if goal.mode == 0:
			for i in range(len(self.goods_pose)):
				self.index = i
				self.robot_pick(self.index)
		
		self.server.set_succeeded(PickResult(final = True))

	def catch_callback(self, goal):
		if goal.command == 0:
			for m in self.markerArray.markers:
				if m.id == self.index:
					m.header.frame_id = '/base_link'
					m.pose.position.x = 0.5
					m.pose.position.y = 0
					m.pose.position.z = 0.25
					rate = rospy.Rate(10.0)
					for i in range(1,10):
						m.pose.position.z = m.pose.position.z + 0.1
						rate.sleep()

		elif goal.command == 1:
			for m in self.markerArray.markers:
				if m.id == self.index:
					m.header.frame_id = '/map'					
					m.pose.position.x = self.robot_pose[0]+0.5
					m.pose.position.y = self.robot_pose[1]
					rate = rospy.Rate(10.0)
					for i in range(1,10):
						m.pose.position.z = m.pose.position.z - 0.1
						rate.sleep()	
		self.catch_server.set_succeeded(CatchResult(final = True))


	def pose_callback(self, msg):
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y

		self.robot_pose = [x, y]
		return


	def set_goods_markers(self):
		self.markerArray = MarkerArray()
		for i in range(len(self.goods_pose)):
			marker = Marker()
			marker.header.frame_id = '/map'
			marker.type = marker.CUBE
			marker.action = marker.ADD
			marker.scale.x = 0.5
			marker.scale.y = 0.5
			marker.scale.z = 0.5
			marker.color.a = 1.0
			marker.color.r = 0.0
			marker.color.g = 1.0
			marker.color.b = 0.0
			marker.pose.orientation.w = 1.0

			marker.pose.position.x = self.goods_pose[i][0]
			marker.pose.position.y = self.goods_pose[i][1]
			marker.pose.position.z = 0.25

			self.markerArray.markers.append(marker)

			id = 0
			for m in self.markerArray.markers:
				m.id = id
				id += 1
		

if __name__ == '__main__':
	rospy.init_node('robot_navigation', argv=sys.argv)
	robot = Robot()
	while not rospy.is_shutdown():
		rate = rospy.Rate(10.0)
		
		robot.publisher.publish(robot.markerArray)
		rate.sleep()
