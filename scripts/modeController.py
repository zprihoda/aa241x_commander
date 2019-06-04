#!/usr/bin/env python

"""
Mode Controller Script for Autonomous Mission
Handles the FSM and high level mission logic

TODO: Validate takeoff altitude (is correct with offset?)
TODO: How do we determine if we've landed
TODO: Implement a distance dependent cutoff
TODO: We may want to switch off once we've landed (disarm)
"""

import rospy
from enum import Enum
import numpy.linalg as npl
import numpy as np

# Import message types
from std_msgs.msg import Int8, Bool, Float32
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State
from aa241x_mission.msg import SensorMeasurement, MissionState
from aa241x_commander.msg import LocalizedBeacons

# Global Variables
TAKEOFF_ALT_THRESHOLD = 10          # Altitude at which we have finished take-off
RETURN_BATTERY_THRESHOLD = 0.20     # battery threshold for returning home
HOME_POS_THRESH = 2.0               # Position error Threshold for determining once we're home
IDLE_TIME = 5.0                     # sit in idle for this long before taking off
MAX_BATTERY_CHARGE = 4400.          # Maximum battery charge in Mah
TARGET_NUM_NODES = 5

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
        self.loc_done = False
        self.search_done = False
        self.mission_complete = False
        self.home_pos = None
        self.beacons_localized = []
        self.new_beacon_detected = False
        self.drone_mode = None      # MANUAL/ALTITUDE/POSITION/OFFBOARD etc.
        self.battery_level = 0.0      # assume we are empty until told otherwise
        self.battery_status = None

        self.e_offset = 0
        self.n_offset = 0
        self.u_offset = 0

        self.pos = None

        # publishers
        self.mode_publisher = rospy.Publisher('/modeController/mode', Int8, queue_size=10)
        self.home_publisher = rospy.Publisher('/modeController/home', Pose, queue_size=10)

        # subscribers
        rospy.Subscriber('/mavros/state', State, self.stateCallback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallback)
        #rospy.Subscriber('/lake_lag_pose', PoseStamped, self.poseCallback)
        rospy.Subscriber('/mavros/battery',BatteryState,self.batteryCallback)
        rospy.Subscriber("/measurement", SensorMeasurement, self.beaconCallback);
        rospy.Subscriber('/navigator/loc_done', Bool, self.locDoneCallback)
        rospy.Subscriber('/navigator/search_done', Bool, self.searchDoneCallback)
        rospy.Subscriber('/localizer/localized_beacons',LocalizedBeacons,self.localizedBeaconCallback)
        rospy.Subscriber('/mission_state',MissionState,self.missionStateCallback)

    ## Callbacks
    def stateCallback(self,msg):
        self.drone_mode = msg.mode

    def poseCallback(self,msg):
        pos = msg.pose.position
        x = pos.x + self.e_offset
        y = pos.y + self.n_offset
        z = pos.z + self.u_offset
        #x = pos.x
        #y = pos.y
        #z = pos.z
        
        if self.home_pos is None and self.e_offset != 0:
            self.home_pos = Pose()
            self.home_pos.position.x = x
            self.home_pos.position.y = y
            self.home_pos.position.z = z
        
        self.pos = msg.pose.position
        self.pos.x = x
        self.pos.y = y
        self.pos.z = z

    def locDoneCallback(self,msg):
        self.loc_done = msg.data

    def searchDoneCallback(self,msg):
        self.search_done = msg.data

    def beaconCallback(self,msg):
        meas_ids = msg.id
        new_ids = list(set(meas_ids)-set(self.beacons_localized))
        if len(new_ids) > 0:
            self.new_beacon_detected = True
        else:
            self.new_beacon_detected = False

    def batteryCallback(self,msg):
        self.battery_status = msg
        self.battery_level = msg.percentage  # 1.0 = full, 0.0 = empty

    def localizedBeaconCallback(self,msg):
        self.beacons_localized = msg.ids

    def missionStateCallback(self,msg):
        self.e_offset = msg.e_offset
        self.n_offset = msg.n_offset
        self.u_offset = msg.u_offset
        #self.e_offset = 0
        #self.n_offset = 0
        #self.u_offset = 0

    ## Decision Functions
    # NOTE: some of these could be replaced by 1 line if statements in determineMode
    #   I opted to make functions for now so we can include more complicated logic if we desire
    def hasInitialized(self):
        check1 = rospy.get_rostime() - self.mode_start_time > rospy.Duration.from_sec(IDLE_TIME)
        check2 = self.e_offset != 0
        check3 = self.battery_status is not None
        check4 = self.home_pos is not None
        return check1 and check2 and check3 and check4

    def hasTakenOff(self):
        return self.pos.z > TAKEOFF_ALT_THRESHOLD

    def newBeaconDetected(self):
        return self.new_beacon_detected

    def localizationFinished(self):
        return self.loc_done

    def searchFinished(self):
        return self.search_done

    def batteryLow(self):
        # maybe: if level <= thresh + dist*scaling
        return self.battery_level <= RETURN_BATTERY_THRESHOLD

    def hasReturnedHome(self):
        cur_pos = np.array([self.pos.x,self.pos.y])
        home_pos = np.array([self.home_pos.position.x,self.home_pos.position.y])
        return npl.norm([home_pos-cur_pos]) <= HOME_POS_THRESH

    def hasLanded(self):
        return False

    ## Main Loop for FSM
    def determineMode(self):
        """ Handle mode changes """

        if not(self.prev_mode == self.mode):
            rospy.loginfo("Current Mode: %s", self.mode)
            self.prev_mode = self.mode
            self.mode_start_time = rospy.get_rostime()

        if self.batteryLow() and self.mode not in [Mode.IDLE, Mode.LANDING, Mode.HOME]:    # we want this to override regardless of mode
            self.mode = Mode.HOME

        elif self.mode == Mode.IDLE:
            if not self.mission_complete and self.drone_mode == "OFFBOARD":
                if self.hasInitialized():
                    self.mode = Mode.TAKEOFF

        elif self.mode == Mode.TAKEOFF:
            if self.hasTakenOff():
                self.mode = Mode.HOME

        elif self.mode == Mode.SEARCH:
            if self.newBeaconDetected():
                self.mode = Mode.LOCALIZATION
            if self.searchFinished():
                self.mode = Mode.HOME

        elif self.mode == Mode.LOCALIZATION:
            self.new_beacon_detected = False
            if self.localizationFinished():
                self.mode = Mode.SEARCH

        elif self.mode == Mode.HOME:
            if self.hasReturnedHome():
                self.mode = Mode.LANDING

        elif self.mode == Mode.LANDING:
            #if not self.hasReturnedHome():
             #   self.mode = Mode.HOME
            if self.hasLanded():
                self.mission_complete = True
                self.mode = Mode.IDLE

    ## Process Functions
    def publish(self):
        """ publish the fsm mode """
        msg = Int8()
        msg.data = self.mode.value
        self.mode_publisher.publish(msg)

        if self.home_pos is not None:
            self.home_publisher.publish(self.home_pos)

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.determineMode()
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    mode_controller = ModeController()
    mode_controller.run()
