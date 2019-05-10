#!/usr/bin/env python

"""
Controller Script for Autonomous Mission
"""

import rospy
import numpy.linalg as npl
from modeController import Mode
from aa241x_commander.msg import Waypoint

# Import message types
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped, Point, Vector3
from mavros_msgs.msg import PositionTarget


class Conroller():
    def __init__(self):
        rospy.init_node('controller', anonymous=True)

        # class variables
        self.mode = None
        self.pos = None     # current drone position

        # setup cmd object
        self.cmd = PositionTarget()
        self.cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.cmd.type_mask = 2499   # command Vx,Vy,Pz
        self.cmd.yaw = 0
        self.cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED    # use the local frame

        self.cmd_pos = Point()  # NOTE: this is defined in ENU
        self.cmd_pos.x = 0  # E
        self.cmd_pos.y = 0  # N
        self.cmd_pos.z = 0  # U

        self.cmd_vel = Vector3() # NOTE: this is defined in ENU
        self.cmd_vel.x = 0  # E
        self.cmd_vel.y = 0  # N
        self.cmd_vel.z = 0  # U

        # publishers
        self.cmd_pub = rospy.Publisher("mavros/setpoint_raw/local",PositionTarget, queue_size=10);

        # subscribers
        rospy.Subscriber('/navigator/waypoint', Waypoint, self.waypointCallback)
        rospy.Subscriber('/modeController/mode',Int8, self.modeCallback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallback)

    ## Callbacks
    def modeCallback(self,msg):
        self.mode = msg.data

    def poseCallback(self,msg):
        self.pos = msg.pose.position

    def waypointCallback(self,msg):
        self.waypoint = msg

    ## Main Loop for Navigator
    def controlLoop(self):

        # Specialized Controllers
        if self.mode == Mode.IDLE:  # do nothing
            pass
        elif self.mode == Mode.TAKEOFF:
            pass
        elif self.mode == Mode.SEARCH:
            pass
        elif self.mode == Mode.LOCALIZATION:
            pass
        elif self.mode == Mode.HOME:
            pass
        elif self.mode == Mode.LANDING:
            pass

    ## Process Functions
    def publish(self):
        """ publish waypoints and navigation messages """
        cmd.header.stamp = rospy.Time.now();
        cmd.position = self.pos;
        cmd.velocity = self.vel;
        cmd_pub.publish(cmd);

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.controlLoop()
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    controller = Controller()
    controller.run()
