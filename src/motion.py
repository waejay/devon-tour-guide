#! usr/bin/env python

import rospy
from threading import Thread, Lock

from geometry_msgs.msg import Twist     # Controls linear/angular velocity

class LinearSpeed:
    NONE   = 0.0
    SLOW   = 0.1
    NORMAL = 0.2
    FAST   = 0.4

class AngularSpeed:
    NONE   = 0.0
    SLOW   = 0.5
    NORMAL = 1.0
    FAST   = 2.0

class Motion:

    def __init__(self):
        rospy.loginfo("Initializing Turtlebot Motion...")

        # Initialize ROS node
        self.name = "Motion object"

        # To change Turtlebot motion...
        self.velocity = Twist()                   # ...set velocity values in
                                                  #    this
        self.publisher = self.set_publisher() # ...and publish using this

        self.mutex = Lock()

        rospy.loginfo("..Turtlebot Motion initialized")

    def set_publisher(self):
        return rospy.Publisher('/mobile_base/commands/velocity', Twist,
                               queue_size = 5)

    def publish(self):
        self.publisher.publish(self.velocity)

    def halt(self):
        self.velocity.linear.x = LinearSpeed.NONE
        self.velocity.angular.z = AngularSpeed.NONE
        self.publish()

    def halt_rotation(self):
        self.velocity.angular.z = AngularSpeed.NONE
        self.publish()

    #  -------------- Linear Motion ------------------

    def move_forward(self, speed = LinearSpeed.NORMAL):
        self.velocity.linear.x = speed
        self.publish()

    def move_backward(self, speed = LinearSpeed.NORMAL):
        self.velocity.linear.x = -1 * speed
        self.publish()

    #  -------------- Angular Motion ------------------

    def rotate_left(self, speed = AngularSpeed.NORMAL):
        self.velocity.angular.z = speed
        self.publish()

    def rotate_right(self, speed = AngularSpeed.NORMAL):
        self.velocity.angular.z = -1 * speed
        self.publish()

    def rotate_180(self, speed = AngularSpeed.NORMAL):

        self.velocity.linear.x = LinearSpeed.NONE
        self.velocity.angular.z = speed

        # Set current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        # Rotate until turned 3.14 rad/180 deg
        while (current_angle < 3.14):
            self.publish()
            t1 = rospy.Time.now().to_sec()
            current_angle = abs(speed)*(t1-t0)

        # Halt motion after angle has been met
        self.halt()









