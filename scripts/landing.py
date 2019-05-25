#!/usr/bin/env python

"""
Landing Command Script for Autonomous Mission
"""

import rospy
import numpy as np
import math
#from modeController import Mode

# Import message types
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped, Point, Vector3, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from aa241x_mission.msg import MissionState
from aa241x_commander.msg import Waypoint
from aa241x_student.msg import tag_info, Targetpoint



class Landing():
    def __init__(self):
        rospy.init_node('landing_node', anonymous=True)

        # class variables
        self.mode = None
        self.pos = None     # current drone position
        self.vel = None
        self.attitude = None

        self.prev_alt = 0   # if no altitude given, maintain previous
        self.drone_mode = None

        self.e_offset = 0
        self.n_offset = 0
        self.u_offset = 0

        self.tag_id = []
        self.tag_position = []
        self.tag_rotation = []

        self.target_x = None
        self.target_y = None
        self.target_z = None
        self.target_yaw = None

        # publishers
        self.target_pub = rospy.Publisher("/landing/target_point", Targetpoint, queue_size=10)

        # subscribers
        #rospy.Subscriber('/modeController/mode',Int8, self.modeCallback)
        #rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallback)
        #rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.velCallback)
        #rospy.Subscriber('/mission_state',MissionState,self.missionStateCallback)
        #rospy.Subscriber('/mavros/state', State, self.stateCallback)
        rospy.Subscriber('/tag_information', tag_info, self.taginfoCallback)


    ## Callbacks
    def stateCallback(self,msg):
        self.current_state = msg
        self.drone_mode = msg.mode

    def modeCallback(self,msg):
        self.mode = Mode(msg.data)

    def poseCallback(self,msg):
        pos = msg.pose.position
        self.pos = np.array([pos.x+self.e_offset, pos.y+self.n_offset])
        self.alt = pos.z + self.u_offset

        quat = msg.pose.orientation
        roll = math.atan2(2.0 * (quat.x * quat.y + quat.z * quat.w), 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))
        pitch = math.asin(2.0 * (quat.x * quat.z - quat.w * quat.y))
        yaw = math.atan2(2.0 * (quat.x * quat.w + quat.y * quat.z), 1.0 - 2.0 * (quat.z * quat.z + quat.w * quat.w))
        self.attitude = np.array([roll, pitch, yaw])

    def velCallback(self,msg):
        vel = msg.twist.linear
        self.vel = np.array([vel.y,vel.x])

    def missionStateCallback(self,msg):
        self.e_offset = msg.e_offset
        self.n_offset = msg.n_offset
        self.u_offset = msg.u_offset

    def taginfoCallback(self, msg):
        self.tag_id = []
        for _id in msg.id:
            self.tag_id.append(_id)
        self.tag_position = dict()
        self.tag_rotation = dict()
        for i in range(len(self.tag_id)):
            _tag_id = self.tag_id[i]
            _tag_position = msg.position[i*3 : (i+1)*3]
            _tag_rotation = msg.rotation[i*9 : (i+1)*9]

            self.tag_position[_tag_id] = _tag_position
            self.tag_rotation[_tag_id] = _tag_rotation


    ## Main Loop for Navigator
    def landcommand(self):
        #if self.mode == Mode.LANDING:
        if len(self.tag_id) != 0:
            center_id = 0
            if center_id in self.tag_id:
                _position = self.tag_position[center_id]
                _rotation = self.tag_rotation[center_id]

            else:
                # use tag formation should be changed
                _position = self.tag_position[self.tag_id[0]]
                _rotation = self.tag_rotation[self.tag_id[0]]
            
            tag_yaw = math.atan2(_rotation[3], _rotation[0])
            tag_pitch = math.atan2(-_rotation[6], math.sqrt(_rotation[7] ** 2 + _rotation[8] ** 2))
            tag_roll = math.atan2(_rotation[7], _rotation[8])
                
            self.target_yaw = tag_yaw
            self.target_x = _position[0]
            self.target_y = _position[1]
            self.target_z = _position[2]

    ## Process Functions
    def publish(self):
        """ publish target point for landing """
        msg = Targetpoint()

        msg.x = self.target_x
        msg.y = self.target_y
        msg.z = self.target_z
        msg.yaw = self.target_yaw

        self.target_pub.publish(msg)


    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.landcommand()
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    landing = Landing()
    landing.run()
