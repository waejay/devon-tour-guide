#! /usr/bin/env python

import rospy

from motion import Motion
from sensor import Sensor
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

    def idle_mode(self):
        rospy.loginfo("Initalizing Idle mode.")
        while not rospy.is_shutdown():
            rospy.loginfo(self.sensor.get_mid_laser_value)
            self.motion.rotate_right()

    def tour_guide_mode(self):
        rospy.loginfo("Initializing Tour Guide mode.")

    def point_to_point_mode(self):
        rospy.loginfo("Initializing P2P Guide mode.")

    def start(self):
        rospy.loginfo("Starting Tour Guide..")

        # TODO: here, could we determine if the guide will be
        #       1) casually walking around,
        #       2) begin P2P mode, or
        #       3) begin tour mode?

        # TODO: for now, it will be 'casually walking around'
        self.idle_mode()

    def shutdown(self):
        rospy.loginfo("Stopping Turtlebot")
        self.motion.halt()
        rospy.sleep(1)          # ensures Turtlebot received the command before
                                # shutting down script

if __name__ == '__main__':
    controller = Controller()

    try:
        controller.start()
    except:
        rospy.loginfO("Controller node terminated.")
