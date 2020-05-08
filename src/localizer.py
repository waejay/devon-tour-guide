#! usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Localizer:

    def __init__(self):
        rospy.loginfo("Initializing Turtlebot Localizer")

        self.position = None

        self.subscribe()

    def subscribe(self):
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback)

    def callback(self, msg):
        self.position = msg.pose.pose.position

    def position(self):
        return self.position

    def moveToGoal(self,goal_position):

	#define a client for to send goal requests to the move_base server through a SimpleActionClient
	ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

	#wait for the action server to come up
	while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
		rospy.loginfo("Waiting for the move_base action server to come up")

	goal = MoveBaseGoal()

	#set up the frame parameters
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()

	# moving towards the goal*/

	goal.target_pose.pose.position = goal_position
	goal.target_pose.pose.orientation.x = 0.0
	goal.target_pose.pose.orientation.y = 0.0
	goal.target_pose.pose.orientation.z = 0.0
	goal.target_pose.pose.orientation.w = 1.0

	rospy.loginfo("Sending goal location ...")
	ac.send_goal(goal)

	ac.wait_for_result(rospy.Duration(60))

	if(ac.get_state() ==  GoalStatus.SUCCEEDED):
		rospy.loginfo("You have reached the destination")
		return True

	else:
		rospy.loginfo("The robot failed to reach the destination")
		return False
