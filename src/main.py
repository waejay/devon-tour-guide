#! /usr/bin/env python

import rospy
import math

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from motion import Motion
from sensor import Sensor, Distance
from localizer import Localizer
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from enum import Enum

class Locations():
    WestEntrance = Point(8.8748, 3.5034, 0)
    EastEntrance = Point(14.7042, -14.4308, 0)
    CSOffice = Point(15.4548, 2.2821, 0)
    Atrium = Point(10.4148, -0.9555, 0)
    ComputerLab = Point(14.0525, -11.699, 0)
    ElectronicsLab = Point(12.0898,-5.444, 0)

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

        self.visited_highlights = []

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
                self.sensor.avoid_obstacle()
            else:
                # If no obstacles detected, no need to rotate
                self.motion.halt_rotation()

            self.motion.move_forward()

    def tour_guide_mode(self):
	''' Robot will decide its path/highlights depending on its location'''
        rospy.loginfo("Initializing Tour Guide mode.")

        print(self.localizer.position)

        west_entrance  = Locations.WestEntrance
        east_entrance  = Locations.EastEntrance

        # TODO: just start at nearest entrance and go down the list
        if self.get_nearest_entrance(west_entrance, east_entrance) == west_entrance:
            self.localizer.moveToGoal(Locations.WestEntrance)
            self.localizer.moveToGoal(Locations.Atrium)
            self.localizer.moveToGoal(Locations.CSOffice)
            self.localizer.moveToGoal(Locations.ElectronicsLab)
            self.localizer.moveToGoal(Locations.ComputerLab)
            self.localizer.moveToGoal(Locations.EastEntrance)
        else:
            self.localizer.moveToGoal(Locations.EastEntrance)
            self.localizer.moveToGoal(Locations.ComputerLab)
            self.localizer.moveToGoal(Locations.ElectronicsLab)
            self.localizer.moveToGoal(Locations.Atrium)
            self.localizer.moveToGoal(Locations.CSOffice)
            self.localizer.moveToGoal(Locations.WestEntrance)


        '''
        # Go to that highlight
        self.moveToGoal(nearest_highlight)

        # Add to "highlights visited"
        visited_highlights.append(nearest_highlight)

        # Keep going until all highlights are visited
        while len(visited_highlights) <= 6:

            # Calculate next highlight
            nearest_highlight = self.get_nearest_highlight()

            # Go to that highlight
            self.moveToGoal(nearest_highlight)

            # Add to "highlights visited"
            visited_highlights.append(nearest_highlight)

        '''
        print("...And that's all! Thank you for participating in the Devon Tour!")

    def get_nearest_entrance(self, west_entrance, east_entrance):
        west_entrance_distance = self.distance(west_entrance)
        east_entrance_distance = self.distance(east_entrance)

        print("distance to west entrance = " + str(west_entrance_distance))
        print("distance to east entrance = " + str(east_entrance_distance))

        if west_entrance_distance < east_entrance_distance:
            return west_entrance
        else:
            return east_entrance

    def distance(self, coordinates):
        distance = math.sqrt((self.localizer.position.x - coordinates.x)**2 +
                             (self.localizer.position.y - coordinates.y)**2)

        return distance

    def point_to_point_mode(self):
	''' User chooses which highlights to go to'''
	choice = self.choose()

	# Depending on which highlight the user chooses, the robot will
	# move to the goal
        if (choice == 1):
            self.goalReached = self.localizer.moveToGoal(Locations.CSOffice)
            print("We've reached the CS Office, where the CS department mainly"
                  + " resides")
        elif (choice == 2):
            self.goalReached = self.localizer.moveToGoal(Locations.Atrium)
            print("Welcome to the Atrium! This area is used for get-togethers,"
                  + "studying, welcome parties, and more!")
        elif (choice == 3):
            self.goalReached = self.localizer.moveToGoal(Locations.EastEntrance)
            print("This is the east entrance")
        elif (choice == 4):
            self.goalReached = self.localizer.moveToGoal(Locations.WestEntrance)
            print("This is the west entrance")
        elif (choice == 5):
            self.goalReached = self.localizer.moveToGoal(Locations.ComputerLab)
            print("Inside this room here is the CS Computer Lab, where any CS"
                  + " student can come in and use the machines or for TA's office"
                  + " hours")
        elif (choice == 6):
            self.goalReached = self.moveToGoal(Locations.ElectronicsLab)
            print("In this room is the Electronics Lab.")

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
                self.goalReached = self.localizer.moveToGoal(Locations.CSOffice)
                print("We've reached the CS Office, where the CS department mainly"
                      + " resides")
            elif (choice == 2):
                self.goalReached = self.localizer.moveToGoal(Locations.Atrium)
                print("Welcome to the Atrium! This area is used for get-togethers,"
                      + "studying, welcome parties, and more!")
            elif (choice == 3):
                self.goalReached = self.localizer.moveToGoal(Locations.EastEntrance)
                print("This is the east entrance")
            elif (choice == 4):
                self.goalReached = self.localizer.moveToGoal(Locations.WestEntrance)
                print("This is the west entrance")
            elif (choice == 5):
                self.goalReached = self.localizer.moveToGoal(Locations.ComputerLab)
                print("Inside this room here is the CS Computer Lab, where any CS"
                      + " student can come in and use the machines or for TA's office"
                      + " hours")
            elif (choice == 6):
                self.goalReached = self.localizer.moveToGoal(Locations.ElectronicsLab)
                print("In this room is the Electronics Lab.")


    def choose(self):
	''' User chooses where they would like to start the tour'''
	tour = 'q'
        rospy.loginfo("Initializing P2P Guide mode.")
        rospy.loginfo("Press a key to go to the highlights:")
        rospy.loginfo("'1': CS/ECE Office")
        rospy.loginfo("'2': Atrium")
        rospy.loginfo("'3': East Entrance")
        rospy.loginfo("'4': West Entrance")
        rospy.loginfo("'5': Computer Lab")
        rospy.loginfo("'6': Electronics Lab")
        rospy.loginfo("'q': Quit")

	tour = input()
	return tour

    def shutdown(self):
        '''Shutdown function that's called when user
           inputs Ctrl + C
        '''
        rospy.loginfo("Stopping Turtlebot")
        self.motion.halt()
        rospy.sleep(1)          # ensures Turtlebot received the command before

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

if __name__ == '__main__':
    controller = Controller()

    try:
        controller.start()
    except Exception as e:
        print(e)
    finally:
        rospy.loginfo("Controller node terminated.")

