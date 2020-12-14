#!/usr/bin/env python

import rospy
import sys
import random
import rospkg

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from rob599_project.srv import SetMode, SetModeResponse
from rob599_project.srv import ShowHouse, ShowHouseResponse
from rob599_project.srv import ShowGoods, ShowGoodsResponse


# randomly generate some goods in the world
# using green marker to represent the goods
# after generation, save all goods position into a file
# display the warehouse area and save each place point into a file

class Generator:
	
	def __init__(self):
		self.pkg_dir = rospkg.RosPack().get_path('rob599_project')

		service = rospy.Service('set_mode', SetMode, self.set_callback)
		rospy.loginfo('/set_mode Service started')

		# generate goods point and save in files 
		self.goods_pose = [[2,1,0],[1,2,1],[4,0,0],[2,3,1],[3,4,0],[4,5,0]]
		self.house_pose = [[4,-2,0],[4,-1,0],[3,-2,0],[3,-1,0],[2,-2,0],[2,-1,0]]
		self.house_point = [[1.5,-2.5],[4.5,-2.5],[4.5,-0.5],[1.5,-0.5]]

		# input variance of goods position, don't set too large
		self.generate_goods(0.1)

		self.save_position(self.goods_pose, 'goods_pose.txt')
		self.save_position(self.house_pose, 'house_pose.txt')
		self.save_position(self.house_point, 'house_point.txt')

		rospy.wait_for_service('show_house')
		self.house_displayer = rospy.ServiceProxy('show_house', ShowHouse)
		rospy.wait_for_service('show_goods')
		self.goods_displayer = rospy.ServiceProxy('show_goods', ShowGoods)

		rospy.spin()
		
	def save_position(self, data, filename):
		path = self.pkg_dir + '/files/'+ filename
		f = open(path, 'w')
		f.write(str(data))
		f.close()

	def set_callback(self, msg):
		try:
			answer = self.house_displayer(msg.command)
			rospy.set_param('task_mode', msg.command)
		except rospy.ServiceException as e:
			rospy.logwarn('Service call failed for {0}: {1}'.format(0, e))

		try:
			answer = self.goods_displayer(msg.command)
		except rospy.ServiceException as e:
			rospy.logwarn('Service call failed for {0}: {1}'.format(0, e))

		return SetModeResponse(True)

	def generate_goods(self, variance):
		for g in self.goods_pose:
			g[0] += variance*random.randint(1,10) - variance*5
			g[1] += variance*random.randint(1,10) - variance*5
		rospy.loginfo('generate goods finished')


if __name__ == '__main__':
	rospy.init_node('set_mode', argv=sys.argv)
	generator = Generator()

