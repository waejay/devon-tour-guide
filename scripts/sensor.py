#! usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan   # Laser data to calculate distance
                                        # from nearby obstacles
from nav_msgs.msg import Odometry       # Positional data relative to Turtlebot

from kobuki_msgs.msg import BumperEvent # Bumper sensor

class Sensor:

    def __init__(self):

        self.lasers = None      # Array of Hokuyo lasers
        self.odom = None        # Odometry
	self.bumper = None 	# Bumper sensor

        self.subscribe()

    def subscribe(self):
        rospy.Subscriber('/laserscan', LaserScan, self.laserscan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
	rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback)

    def laserscan_callback(self, laser_msg):
        '''Retrieves LaserScan topic's data'''
        self.lasers = laser_msg

    def odom_callback(self, odom_msg):
        '''Retrieves Odometry topic's data'''
        self.odom = odom_msg

    def get_laser_values(self):
        '''Returns an array of laser values'''
        return self.lasers

    def get_left_laser_value(self):
        return self.lasers.ranges[250]

    def get_mid_laser_value(self):
        return self.lasers.ranges[180]

    def get_right_laser_value(self):
        return self.lasers.ranges[110]

    def get_position(self):
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y

        return (x,y)

	
    def is_bumped(self):
	''' Return the bumper when bumped'''
	if self.bumper.state == BumperEvent.PRESSED:
		return self.bumper
