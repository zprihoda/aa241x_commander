#!/usr/bin/env python

"""
Mode Controller Script for Autonomous Mission
"""

import rospy
from enum import Enum

# Import message types
from std_msgs.msg import Int8, Bool
from geometry_msgs.msg import Pose, PoseStamped
from mavros_msgs import State


class Mode(Enum):
    """State machine modes"""
    IDLE         = 1
    TAKEOFF      = 2
    SEARCH       = 3
    LOCALIZATION = 4
    HOME         = 5
    LANDING      = 6

class ModeController():

    def __init__(self):
        rospy.init_node('mode_controller', anonymous=True)

        self.mode = Mode.IDLE
        self.prev_mode = None
        self.nav_done = False

        # publishers
        self.mode_publisher = rospy.Publisher('/modeController/mode', Int8, queue_size=10)

        # subscribers
        rospy.Subscriber('/mavros/state', State, self.stateCallback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallback)

        """Naviagator will publish a nav_done mesage when the current navigation task is complete
        Examples: Once we have reached taken off (reached a certain altitude), once we are done localizing,
        once we have returned home, etc."""
        rospy.Subscriber('/navigator/loc_done', Bool, self.locDoneCallback)

    def stateCallback(self,msg):
        self.current_state = msg;

    def poseCallback(self,msg):
        self.pose_header = msg.header
        self.pose = msg.pose

    def navDoneCallback(self,msg):
        self.nav_done = True

    def determineMode(self):
        """ Handle mode changes """

        if not(self.prev_mode == self.mode):
            rospy.loginfo("Current Mode: %s", self.mode)
            self.prev_mode = self.mode
            self.prev_mode_time = rospy.get_rostime()

        if self.mode == Mode.IDLE:
            # TODO: When do we switch out of idle
            pass
        elif self.mode == Mode.TAKEOFF:
            pass
        elif self.mode == Mode.SEARCH:
            pass
        elif self.mode == Mode.HOME:
            pass
        elif self.mode == Mode.LANDING:
            pass


    def publish(self):
        """ publish the fsm mode """
        msg = Int8()
        msg.data = self.mode
        self.mode_publisher.publish(msg)

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.determineMode()
            self.publish()
            rate.sleep()

if __name__ == '__main__':
    mode_controller = ModeController()
    mode_controller.run()
