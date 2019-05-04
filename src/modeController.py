#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from std_msgs.msg import Int8
from enum import Enum


class Mode(Enum):
    """State machine modes"""
    IDLE    = 1
    TAKEOFF = 2
    SEARCH  = 3
    HOME    = 4
    LANDING = 5

class ModeController():

    def __init__(self):
        rospy.init_node('mode_controller', anonymous=True)

        self.mode = Mode.IDLE

        # publishers
        self.mode_publisher = rospy.Publisher('/fsm_mode', Int8, queue_size=10)

        # subscribers
        # maybe position (depending on where we get it from).


    def determineMode(self):
        """ Handle mode changes """
        pass

    def publish(self):
        """ publish the fsm mode """
        pass

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.determineMode()
            self.publish()
            rate.sleep()

if __name__ == '__main__':
    mode_controller = ModeController()
    mode_controller.run()


