#!/usr/bin/env python

"""
Mode Controller Script for Autonomous Mission
"""

import rospy
from enum import Enum
import numpy.linalg as npl

# Import message types
from std_msgs.msg import Int8, Bool, Float32
from geometry_msgs.msg import Pose, PoseStamped
from mavros_msgs import State


# Global Variables
TAKEOFF_ALT_THRESHOLD = 30
RETURN_BATTERY_THRESHOLD = 0.20   # battery threshold for returning home
HOME_POS_THRESH = 1.0

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
        self.mission_complete = False

        self.home_pos = None


        # publishers
        self.mode_publisher = rospy.Publisher('/modeController/mode', Int8, queue_size=10)

        # subscribers
        rospy.Subscriber('/mavros/state', State, self.stateCallback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallback)


        """Naviagator will publish a nav_done mesage when the current navigation task is complete
        Examples: Once we have reached taken off (reached a certain altitude), once we are done localizing,
        once we have returned home, etc."""
        rospy.Subscriber('/navigator/loc_done', Bool, self.locDoneCallback)

        # TODO: What is the message type here
        rospy.Subscriber('/beaconDetects',Bool,self.beaconCallback)

        # TODO: Where do we get battery level from?
        rospy.Subscriber('/batteryLevel',Float32,self.batteryCallback)

    ## Callbacks
    def stateCallback(self,msg):
        self.current_state = msg;

    def poseCallback(self,msg):
        if self.home_pos is None:
            pos = msg.pose.position
            self.home_pos = np.array([pos.x,pos.y,TAKEOFF_ALT_THRESHOLD])
        self.pose_header = msg.header
        self.pose = msg.pose

    def locDoneCallback(self,msg):
        self.loc_done = True

    def beaconCallback(self,msg):
        # TODO: determine whether any beacons detected are new
        self.new_beacon_detected = True

    def batteryCallback(self,msg):
        self.battery_status = msg


    ## Decision Functions
    # NOTE: some of these could be replaced by 1 line if statements in determineMode
    #   I opted to make functions for now so we can include more complicated logic if we desire
    def hasTakenOff(self):
        # TODO: Does z correctly measure altitude? (it might measure some relative pose)
        return self.pose.position.z > TAKEOFF_ALT_THRESHOLD:

    def newBeaconDetected(self):
        return self.new_beacon_detected:

    def localizationFinished(self):
        return self.loc_done

    def searchFinished(self):
        # TODO: where are we getting nodes_localized (should be published by beaconLocalization script)
        return self.nodes_localized >= TARGET_NUM_NODES:

    def batteryLow(self):
        # TODO: Implement a distance dependent cutoff
        # maybe: if level <= thresh + dist*scaling
        return self.battery_status.battery_level <= RETURN_BATTERY_THRESHOLD:

    def hasReturnedHome(self):
        pos = self.pose.position
        cur_pos = np.array([pos.x,pos.y,pos.z])
        return npl.norm([self.home_pose-cur_pos]) <= HOME_POS_THRESH:

    def hasLanded(self):
        # TODO: How do we determine if we've landed
        return False

    ## Main Loop for FSM
    def determineMode(self):
        """ Handle mode changes """

        if not(self.prev_mode == self.mode):
            rospy.loginfo("Current Mode: %s", self.mode)
            self.prev_mode = self.mode
            self.mode_start_time = rospy.get_rostime()

        if self.mode == Mode.IDLE:
            if not self.mission_complete and self.current_state.mode == "OFFBOARD":
                self.mode = Mode.TAKEOFF

        elif self.mode == Mode.TAKEOFF:
            if self.hasTakenOff():
                self.mode = Mode.SEARCH

        elif self.mode == Mode.SEARCH:
            if self.newBeaconDetected():
                self.mode = Mode.LOCALIZATION
            if self.searchFinished() or self.batteryLow():
                self.mode = Mode.HOME

        elif self.mode == Mode.LOCALIZATION:
            if self.localizationFinished():
                self.mode = Mode.SEARCH

        elif self.mode == Mode.HOME:
            if self.hasReturnedHome():
                self.mode = Mode.LANDING

        elif self.mode == Mode.LANDING:
            # TODO: We may want to switch off once we've landed (disarm)
            if self.hasLanded():
                self.mission_complete = True
                self.mode = Mode.IDLE

    ## Process Functions
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
