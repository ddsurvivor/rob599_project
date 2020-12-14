#!/usr/bin/env python

import rospy
import sys
import rospkg

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

from rob599_project.srv import ShowHouse, ShowHouseResponse
from rob599_project.srv import ShowGoods, ShowGoodsResponse


class ShowMarker:
	
	def __init__(self):
		self.pkg_dir = rospkg.RosPack().get_path('rob599_project')

		self.goods_service = rospy.Service('show_goods', ShowGoods, self.goods_callback)
		rospy.loginfo('/show_goods Service started')

		self.house_service = rospy.Service('show_house', ShowHouse, self.house_callback)
		rospy.loginfo('/show_house Service started')

		self.goods_pose = self.load_data('goods_pose.txt')
		rospy.loginfo(self.goods_pose)
		self.house_point = self.load_data('house_point.txt')

	def goods_callback(self, msg):
		markerArray = MarkerArray()
		for i in range(len(self.goods_pose)):			
			marker = Marker()
			marker.header.frame_id = "/map"
			marker.type = marker.CUBE
			marker.action = marker.ADD
			marker.scale.x = 0.5
			marker.scale.y = 0.5
			marker.scale.z = 0.5
			marker.color.a = 1.0
			marker.color.r = 0.0

			if msg.command == 0:			
				marker.color.g = 1.0
				marker.color.b = 0.0
			else:
				marker.color.g = 1.0 - self.goods_pose[i][2]
				marker.color.b = 0.0 + self.goods_pose[i][2]

			marker.pose.orientation.w = 1.0
			marker.pose.position.x = self.goods_pose[i][0]
			marker.pose.position.y = self.goods_pose[i][1]
			marker.pose.position.z = 0.25

			markerArray.markers.append(marker)

			id = 0
			for m in markerArray.markers:
				m.id = id
				id += 1

		publisher = rospy.Publisher('goods_box_array', MarkerArray, queue_size = 10)
		publisher.publish(markerArray)

		return ShowGoodsResponse(True)

	def house_callback(self, msg):
		line = Marker()
		line.header.frame_id ='/map'
		line.type = line.LINE_STRIP
		line.action = line.ADD

		line.scale.x = 0.1
		line.color.r = 0.0
		line.color.g = 1.0
		line.color.b = 0.0
		line.color.a = 1.0
		for i in range(len(self.house_point)):
			point = Point()
			point.x = self.house_point[i][0]
			point.y = self.house_point[i][1]
			line.points.append(point)

		line.pose.orientation.w = 1.0

		marker_publisher = rospy.Publisher('house_line', Marker, queue_size=10)

		marker_publisher.publish(line)

		# publish another warehouse
		if msg.command == 1:
			line = Marker()
			line.header.frame_id ='/map'
			line.type = line.LINE_STRIP
			line.action = line.ADD

			line.scale.x = 0.1
			line.color.r = 0.0
			line.color.g = 0.0
			line.color.b = 1.0
			line.color.a = 1.0
			for i in range(len(self.house_point)):
				point = Point()
				point.x = self.house_point[i][0]
				point.y = self.house_point[i][1] - 2.5
				line.points.append(point)

			line.pose.orientation.w = 1.0

			marker_publisher = rospy.Publisher('house_line2', Marker, queue_size=10)

			marker_publisher.publish(line)
				

		return ShowHouseResponse(True)

	def load_data(self, filename):
		path = self.pkg_dir + '/files/'+ filename
		f = open(path, 'r')
		data = eval(f.read())
		f.close()
		rospy.loginfo('load data {0}'.format(filename))
		return data		


if __name__ == '__main__':
	rospy.init_node('show_marker', argv=sys.argv)
	showMarker = ShowMarker()
	rospy.spin()
