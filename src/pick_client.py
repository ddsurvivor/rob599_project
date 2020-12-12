#!/usr/bin/env python3



import rospy
import actionlib
import sys

# import pick action
from rob599_project.msg import PickAction, PickGoal, PickFeedback, PickResult



# client call back
def done_callback(status, result):
	if status == actionlib.GoalStatus.SUCCEEDED:
		rospy.loginfo('Suceeded with result {0}'.format(result.final))
	else:
		rospy.loginfo('Failed with result {0}'.format(result.final))


def active_callback():
	rospy.loginfo('Action is active')	


def feedback_callback(feedback):
	rospy.loginfo('Feedback: Go to: {0}'.format(feedback.index))


if __name__ == '__main__':
	# Initialize the node
	rospy.init_node('pick_client', argv=sys.argv)

	pick_client = actionlib.SimpleActionClient('pick_action', PickAction)
	pick_client.wait_for_server()


	goal = PickGoal()
	goal.mode = 0  # set a goal to as mode 0

	pick_client.send_goal(goal, done_cb=done_callback, active_cb=active_callback, feedback_cb=feedback_callback)
	pick_client.wait_for_result()

