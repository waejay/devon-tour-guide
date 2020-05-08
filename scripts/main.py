#! /usr/bin/env python

import rospy

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from motion import Motion
from sensor import Sensor, Distance
from localizer import Localizer
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point

class Controller:
    '''Driver class for controlling tour guide's hybrid paradigm'''

    def __init__(self):

        # Initialize ROS node
        rospy.init_node("Controller")
        self.name = "Controller object"
        self.rate = rospy.Rate(10)          # allow node to 'spin' at 10hz

        rospy.on_shutdown(self.shutdown)    # calls this function when Ctrl+C

        self.motion = Motion()
        self.sensor = Sensor()
        self.localizer = Localizer()

        ''' TODO: old highlights, remove if needed
        # Highlight coordinates
        self.xWestEntrance = -10.5329
        self.yWestEntrance = -3.8948
        self.xEastEntrance = 12.3739
        self.yEastEntrance = -3.7555
        self.xCsOffice = -6.3712
        self.yCsOffice = 2.5169
        self.xAtrium = 3.5468
        self.yAtrium = -4.2581
        '''

        # Highlight coordinates based on amcl_pose + rviz
        self.xWestEntrance = 8.888
        self.yWestEntrance = 3.4802
        self.xEastEntrance = 14.7042
        self.yEastEntrance = -14.4308
        self.xCsOffice = 15.4548
        self.yCsOffice = 2.2821
        self.xAtrium = 10.4148
        self.yAtrium = -0.9555

        # Boolean for reached goal
        self.goalReached = False

    def start(self):
        rospy.loginfo("Starting Tour Guide..")
        begin = 'q'

        # Note: the Motion, Sensor, and Localizer classes
        # seem to take a second to fully initialize so we give
        # this Controller a second to fully initialize as well
        rospy.sleep(1)

        # TODO: here, could we determine if the guide will be
        #       1) casually walking around,
        #       2) begin P2P mode, or
        #       3) begin tour mode?

        # TODO: for now, it will be 'casually walking around'
		# TODO: Chris -  I was thinking we should just take out 'idle_mode'
		# for demo purposes. As soon as the program starts, we can ask the user
		# if they would like a P2P or a regular tour
		# then go from there
        # self.idle_mode()

		# TODO: Chris -  I think it will also be easier if we just have the robot
		# start the tour at its initial position which is at the West Entrance
	rospy.loginfo("Would you like a P2P tour or a regular tour?")
	rospy.loginfo("Press a key:")
	rospy.loginfo("'1': Point To Point")
	rospy.loginfo("'2': Regular Tour")
	begin = input()

        if (begin == 1):
		self.point_to_point_mode()
        elif (begin == 2):
		self.tour_guide_mode()

    def idle_mode(self):
        '''Make Turtlebot move around aimlessly
           and avoid nearby obstacles
        '''
        rospy.loginfo("Initalizing Idle mode.")

        while not rospy.is_shutdown():

            if self.sensor.is_obstacle_detected():
                # Rotate Turtlebot accordingly to avoid obstacles
                self.avoid_obstacle()
            else:
                # If no obstacles detected, no need to rotate
                self.motion.halt_rotation()

            self.motion.move_forward()

	''' Robot will decide its path/highlights depending on its location'''
	# TODO: Chris - This will require automation so I have not done it yet
    def tour_guide_mode(self):
        rospy.loginfo("Initializing Tour Guide mode.")

	''' User chooses which highlights to go to'''
    def point_to_point_mode(self):
	choice = self.choose()

	# Depending on which highlight the user chooses, the robot will
	# move to the goal
        if (choice == 1):
            self.goalReached = self.moveToGoal(self.xCsOffice, self.yCsOffice)
        elif (choice == 2):
            self.goalReached = self.moveToGoal(self.xAtrium, self.yAtrium)
        elif (choice == 3):
            self.goalReached = self.moveToGoal(self.xEastEntrance, self.yEastEntrance)

	# if choice isn't q and the robot reached its goal
	if (choice != 'q'):
            if (self.goalReached):
                rospy.loginfo("Reached highlight!")
            # If it fails to reach the goal
            else:
                rospy.loginfo("Couldn't reach the highlight, try again")

		# Loop to keep going until user quits the tour
	while choice != 'q':
            choice = self.choose()
            if (choice == 1):
                self.goalReached = self.moveToGoal(self.xCsOffice, self.yCsOffice)
            elif (choice == 2):
                self.goalReached = self.moveToGoal(self.xAtrium, self.yAtrium)
            elif (choice == 3):
                self.goalReached = self.moveToGoal(self.xEastEntrance, self.yEastEntrance)


	''' User chooses where they would like to start the tour'''
    def choose(self):
	tour = 'q'
        rospy.loginfo("Initializing P2P Guide mode.")
        rospy.loginfo("Press a key to go to the highlights:")
        rospy.loginfo("'1': CS/ECE Office")
        rospy.loginfo("'2': Atrium")
        rospy.loginfo("'3': East Entrance")
        rospy.loginfo("'q': Quit")

	tour = input()
	return tour

	# TODO: Chris - I copied this from the gaitech "Map-Based Navigation" websitfor inspiration hoping that this code base could miracuously work for our project. I think this could give a head-start on what we'd have to do for P2P at least
    def moveToGoal(self,xGoal,yGoal):

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

	goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
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



    def avoid_obstacle(self):
        '''Checks where a nearby obstacle is and
           rotates accordingly
        '''
        left_laser = self.sensor.get_left_laser_value()
        mid_laser = self.sensor.get_mid_laser_value()
        right_laser = self.sensor.get_right_laser_value()

        # Check if obstacle in front is 1 ft away
        if mid_laser <= Distance.FOOT:
            self.motion.rotate_180()
        # Check if obstacle is 2 ft to the left
        elif left_laser < 2 * Distance.FOOT:
            self.motion.rotate_right()
        # Check if obstacle is 2 ft to the right
        elif right_laser < 2 * Distance.FOOT:
            self.motion.rotate_left()

    def shutdown(self):
        '''Shutdown function that's called when user
           inputs Ctrl + C
        '''
        rospy.loginfo("Stopping Turtlebot")
        self.motion.halt()
        rospy.sleep(1)          # ensures Turtlebot received the command before

if __name__ == '__main__':
    controller = Controller()

    try:
        controller.start()
    except Exception as e:
        print(e)
    finally:
        rospy.loginfo("Controller node terminated.")

