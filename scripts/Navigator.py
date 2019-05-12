#!/usr/bin/env python

"""
Navigator Script for Autonomous Mission
Handles pathplanning and setting of waypoints
"""

import rospy
import numpy.linalg as npl
import numpy as np
from modeController import Mode
from searchPath import search_path

# Import message types
from std_msgs.msg import Int8, Bool, Float32
from geometry_msgs.msg import Pose, PoseStamped
from aa241x_mission.msg import SensorMeasurement
from aa241x_commander.msg import Waypoint, LocalizedBeacons
from aa241x_mission.msg import MissionState


SEARCH_ALT = 50 # desired altitude for performing search mode
LOCALIZE_ALT = 30   # desired altitude for performing localization
PATH_THRESH = 1     # move to next path once within 5 meters of end point (smooths out path)

class Navigator():
    def __init__(self):
        rospy.init_node('navigator', anonymous=True)

        # class variables
        self.home_pos = None
        self.localized_beacons = []
        self.unlocalized_beacons = {}   # id : [n,e]  (measured_pos)
        self.mode = None

        self.search_wp_idx = 0
        self.waypoint_n = []
        self.waypoint_e = []
        self.waypoint_alt = []
        self.loc_done = False
        self.search_done = False

        self.e_offset = 0
        self.n_offset = 0
        self.u_offset = 0

        # publishers
        self.waypoint_pub = rospy.Publisher('/navigator/waypoint', Waypoint, queue_size=10)
        self.loc_done_pub = rospy.Publisher('/navigator/loc_done', Bool, queue_size=10)
        self.search_done_pub = rospy.Publisher('/navigator/search_done', Bool, queue_size=10)

        # subscribers
        rospy.Subscriber('/modeController/mode',Int8, self.modeCallback)
        rospy.Subscriber('/modeController/home',Pose, self.homeCallback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallback)
        rospy.Subscriber("/measurement", SensorMeasurement, self.beaconCallback);
        rospy.Subscriber('/localizer/localized_beacons',LocalizedBeacons,self.localizedBeaconCallback)
        rospy.Subscriber('/mission_state',MissionState,self.missionStateCallback)


    ## Callbacks
    def modeCallback(self,msg):
        self.mode = Mode(msg.data)

    def poseCallback(self,msg):
        pos = msg.pose.position
        x = pos.x + self.e_offset
        y = pos.y + self.n_offset
        z = pos.z + self.u_offset

        self.pose_header = msg.header
        self.pos = msg.pose.position
        self.pos.x = x
        self.pos.y = y
        self.pos.z = z

    def homeCallback(self,msg):
        self.home_pos = np.array([msg.position.x,msg.position.y,msg.position.z])

    def localizedBeaconCallback(self,msg):
        self.localized_beacons = msg.ids
        for id in msg.ids:
            self.unlocalized_beacons.pop(id,None)   # remove from unlocalized_beacons if its present

    def beaconCallback(self,msg):
        meas_ids = msg.id
        for i in range(len(meas_ids)):
            if meas_ids[i] in self.localized_beacons:
                continue
            elif meas_ids[i] in self.unlocalized_beacons.keys():
                continue
            else:
                self.unlocalized_beacons[meas_ids[i]] = np.array([msg.n[i],msg.e[i]])

    def missionStateCallback(self,msg):
        self.e_offset = msg.e_offset
        self.n_offset = msg.n_offset
        self.u_offset = msg.u_offset



    ## Main Loop for Navigator
    def navigate(self):

        if self.mode == Mode.IDLE:  # no waypoint
            self.waypoint_e = []
            self.waypoint_n = []
            self.waypoint_alt = []

        elif self.mode == Mode.TAKEOFF:
            self.waypoint_e = [self.home_pos[0]]
            self.waypoint_n = [self.home_pos[1]]
            self.waypoint_alt = [40]  # set altitude waypoint above 30

        elif self.mode == Mode.SEARCH:
            # TODO: implement search mode navigation
            if self.search_wp_idx >= len(search_path):
                self.search_done = True
                self.waypoint_e = []
                self.waypoint_n = []
                self.waypoint_alt = []
            else:
                wp = np.array(search_path[self.search_wp_idx])
                pos = np.array([self.pos.x, self.pos.y])

                if self.search_wp_idx == 0:     # set waypoint as a single point
                    self.waypoint_e = [self.home_pos[0],wp[0]]
                    self.waypoint_n = [self.home_pos[1],wp[1]]
                    self.waypoint_alt = [SEARCH_ALT]
                else:   # set waypoint as a path
                    wp = np.array(search_path[self.search_wp_idx])
                    wp_prev = np.array(search_path[self.search_wp_idx-1])
                    self.waypoint_e = [wp_prev[0],wp[0]]
                    self.waypoint_n = [wp_prev[1],wp[1]]
                    self.waypoint_alt = [SEARCH_ALT,SEARCH_ALT]

                if npl.norm(wp-pos) < PATH_THRESH:
                    self.search_wp_idx += 1

        elif self.mode == Mode.LOCALIZATION:
            if len(self.unlocalized_beacons.keys()) == 0:
                self.loc_done = True
            else:
                self.loc_done = False
                beacon_id = sorted(self.unlocalized_beacons.keys())[0]
                beacon_pos = self.unlocalized_beacons[beacon_id]
                self.waypoint_e = [beacon_pos[0]]
                self.waypoint_n = [beacon_pos[1]]
                self.waypoint_alt = [LOCALIZE_ALT]

        elif self.mode == Mode.HOME:
            self.waypoint_e = [self.home_pos[0]]
            self.waypoint_n = [self.home_pos[1]]
            self.waypoint_alt = [30]

        elif self.mode == Mode.LANDING:
            self.waypoint_e = [self.home_pos[0]]
            self.waypoint_n = [self.home_pos[1]]
            self.waypoint_alt = []


    ## Process Functions
    def publish(self):
        """ publish waypoints and navigation messages """
        msg = Waypoint()
        msg.e = self.waypoint_e
        msg.n = self.waypoint_n
        msg.alt = self.waypoint_alt
        self.waypoint_pub.publish(msg)

        msg = Bool()
        msg.data = self.loc_done
        self.loc_done_pub.publish(msg)

        msg = Bool()
        msg.data = self.search_done
        self.search_done_pub.publish(msg)


    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.navigate()
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    navigator = Navigator()
    navigator.run()
