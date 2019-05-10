#!/usr/bin/env python

"""
Navigator Script for Autonomous Mission
Handles pathplanning and setting of waypoints
"""

import rospy
import numpy.linalg as npl
from modeController import Mode
from aa241x_commander.msg import Waypoint

# Import message types
from std_msgs.msg import Int8, Bool, Float32
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State
from aa241x_mission.msg import SensorMeasurement


SEARCH_ALT = 50 # desired altitude for performing search mode
LOCALIZE_ALT = 30   # desired altitude for performing localization

class Navigator():
    def __init__(self):
        rospy.init_node('navigator', anonymous=True)

        # class variables
        self.home_pos = None
        self.localized_beacons = []
        self.unlocalized_beacons = {}   # id : [n,e]  (measured_pos)
        self.mode = None

        self.waypoint_n = None
        self.waypoint_e = None
        self.waypoint_alt = None
        self.loc_done = False

        # publishers
        self.waypoint_pub = rospy.Publisher('/navigator/waypoint', Waypoint, queue_size=10)
        self.loc_done_pub = rospy.Publisher('/navigator/loc_done', Bool, queue_size=10)

        # subscribers
        rospy.Subscriber('/modeController/mode',Int8, self.modeCallback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallback)
        rospy.Subscriber("/measurement", SensorMeasurement, self.beaconCallback);
        rospy.Subscriber('/localizer/localized_beacons',Bool,self.localizedBeaconCallback)


    ## Callbacks
    def modeCallback(self,msg):
        self.mode = msg.data

    def poseCallback(self,msg):
        if self.home_pos is None:
            pos = msg.pose.position
            self.home_pos = np.array([pos.x,pos.y,TAKEOFF_ALT_THRESHOLD])
        self.pose_header = msg.header
        self.pose = msg.pose

    def localizedBeaconCallback(self,msg):
        self.localized_beacons = msg.ids
        for id in msg.ids:
            self.unlocalized_beacons.pop(id,None)   # remove from unlocalized_beacons if its present

    def beaconCallback(self,msg):
        meas_ids = msg.id
        for i in range(len(meas_ids)):
            if meas_ids[i] in localized_beacons:
                continue
            elif meas_ids[i] in unlocalized_beacons.keys():
                continue
            else:
                unlocalized_beacons[meas_ids[i]] = np.array([msg.n[i],msg.e[i]])

    ## Main Loop for Navigator
    def navigate(self):

        if self.mode == Mode.IDLE:  # no waypoint
            self.waypoint_n = None
            self.waypoint_e = None
            self.waypoint_alt = None


        elif self.mode == Mode.TAKEOFF:
            self.waypoint_n = self.home_pos[0]
            self.waypoint_e = self.home_pos[1]
            self.waypoint_alt = 40  # set altitude waypoint above 30

        elif self.mode == Mode.SEARCH:
            # TODO: implement search mode navigation
            pass

        elif self.mode == Mode.LOCALIZATION:
            # TODO: implement localization mode navigation
            pass

        elif self.mode == Mode.HOME:
            self.waypoint_n = self.home_pos[0]
            self.waypoint_e = self.home_pos[1]
            self.waypoint_alt = 30

        elif self.mode == Mode.LANDING:
            self.waypoint_n = self.home_pos[0]
            self.waypoint_e = self.home_pos[1]
            self.waypoint_alt = None


    ## Process Functions
    def publish(self):
        """ publish the fsm mode """
        msg = Waypoint()
        msg.n = self.waypoint_n
        msg.e = self.waypoint_e
        msg.alt = self.waypoint_alt
        self.waypoint_pub.publish(msg)

        msg = Bool()
        msg.data = self.loc_done
        self.loc_done_pub.publish(msg)

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.navigate()
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    navigator = Navigator()
    navigator.run()
