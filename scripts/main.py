#! /usr/bin/env python

import rospy

from motion import Motion
from sensor import Sensor, Distance
from localizer import Localizer

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

    def start(self):
        rospy.loginfo("Starting Tour Guide..")

        # Note: the Motion, Sensor, and Localizer classes
        # seem to take a second to fully initialize so we give
        # this Controller a second to fully initialize as well
        rospy.sleep(1)

        # TODO: here, could we determine if the guide will be
        #       1) casually walking around,
        #       2) begin P2P mode, or
        #       3) begin tour mode?

        # TODO: for now, it will be 'casually walking around'
        self.idle_mode()

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

    def tour_guide_mode(self):
        rospy.loginfo("Initializing Tour Guide mode.")

    def point_to_point_mode(self):
        rospy.loginfo("Initializing P2P Guide mode.")

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

