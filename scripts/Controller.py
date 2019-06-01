#!/usr/bin/env python

"""
Controller Script for Autonomous Mission
TODO: implement landing controller
"""

import rospy
import numpy as np
import math
import numpy.linalg as npl
from modeController import Mode

# Import message types
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped, Point, Vector3, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from aa241x_mission.msg import MissionState
from aa241x_commander.msg import Waypoint, Targetpoint


V_MAX = 5.0     # in m/s
VEL_CONTROL_MASK = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
                    PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                    PositionTarget.IGNORE_YAW_RATE);


def pointController(p_des,p_cur,v_cur):
    kp = 1.0
    kd = 0.5
    delta_p = p_des-p_cur
    v_cmd = kp*delta_p - kd*v_cur

    if npl.norm(v_cmd) > V_MAX:
        v_cmd = v_cmd/npl.norm(v_cmd) * V_MAX

    return v_cmd

def pathController(p1,p2,p_cur,v_cur):

    if np.linalg.norm(p1 - p2) == 0:
        p1 = p_cur
    direction_vector = (p2 - p1)/np.linalg.norm(p2 - p1)

    if direction_vector[1] == 0:
        coeff_a = 0
        coeff_b = 1
    else:
        coeff_a = 1
        coeff_b = - direction_vector[0] * coeff_a / direction_vector[1]

    coeff_c = - coeff_a * p1[0] - coeff_b * p1[1]
    error = -(coeff_a * p_cur[0] + coeff_b * p_cur[1] + coeff_c) / np.sqrt(coeff_a **2 + coeff_b **2)
    normal_vector = np.array([coeff_a, coeff_b])
    normal_vector = normal_vector/npl.norm(normal_vector)

    r_perp = error
    rdot_perp = np.inner(v_cur, normal_vector)
    n_par = direction_vector
    n_perp = normal_vector

    k_p = 1
    k_d = 0.5
    v_perp = (k_p*r_perp + k_d*rdot_perp) * n_perp
    v_par = V_MAX*n_par

    # if we are past p2 along path, reverse direction
    if np.dot(p2-p_cur,direction_vector) < 0 :
        v_par *= -1

    g = 1.0     # v_cmd = g*v_par + v_perp
    v_cmd = g*v_par + v_perp
    v_cmd = v_cmd/npl.norm(v_cmd) * V_MAX
    yaw_cmd = np.arctan2(v_cmd[1],v_cmd[0])

    return v_cmd, yaw_cmd


class Controller():
    def __init__(self):
        rospy.init_node('controller', anonymous=True)

        # class variables
        self.mode = None
        self.pos = None     # current drone position
        self.vel = None
        self.vel_alt = None
        self.prev_alt = 0   # if no altitude given, maintain previous
        self.waypoint = None
        self.drone_mode = None

        self.e_offset = 0
        self.n_offset = 0
        self.u_offset = 0

        self.tag_point_x = None
        self.tag_point_y = None
        self.tag_point_z = None
        self.tag_point_yaw = None

        # setup cmd object
        self.cmd = PositionTarget()
        self.cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        self.cmd.type_mask = VEL_CONTROL_MASK   # command Vx,Vy,Vz,Yaw
        self.cmd.yaw = 0

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
        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.velCallback)
        rospy.Subscriber('/mission_state',MissionState,self.missionStateCallback)
        rospy.Subscriber('/landing/target_point', Targetpoint, self.targetpointCallback)


    ## Callbacks
    def modeCallback(self,msg):
        self.mode = Mode(msg.data)

    def poseCallback(self,msg):
        pos = msg.pose.position
        self.pos = np.array([pos.x+self.e_offset, pos.y+self.n_offset])
        self.pos = np.array([pos.x, pos.y])
        self.alt = pos.z + self.u_offset
        self.alt = pos.z

    def globalPoseCallback(self.msg):
        quat = msg.pose.orientation
        roll = math.atan2(2.0 * (quat.x * quat.y + quat.z * quat.w), 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))
        pitch = math.asin(2.0 * (quat.x * quat.z - quat.w * quat.y))
        yaw = math.atan2(2.0 * (quat.x * quat.w + quat.y * quat.z), 1.0 - 2.0 * (quat.z * quat.z + quat.w * quat.w))
        self.attitude = np.array([roll, pitch, yaw])

    def velCallback(self,msg):
        vel = msg.twist.linear
        self.vel = np.array([vel.y,vel.x])
        self.vel_alt = vel.z 

    def waypointCallback(self,msg):
        if len(msg.alt) > 0:
            self.waypoint = msg
            self.prev_alt = msg.alt[-1]
        else:
            self.waypoint = msg
            self.waypoint.alt == [self.prev_alt]

    def missionStateCallback(self,msg):
        self.e_offset = msg.e_offset
        self.n_offset = msg.n_offset
        self.u_offset = msg.u_offset

    def targetpointCallback(self, msg):
        self.tag_point_x = msg.x
        self.tag_point_y = msg.y
        self.tag_point_z = msg.z
        self.tag_point_yaw = msg.yaw


    ## Main Loop for Navigator
    def controlLoop(self):

        # default behavior (0 xyz velocity)
        self.cmd_vel.x = 0
        self.cmd_vel.y = 0
        self.cmd_vel.z = 0
        self.cmd_yaw = 0

        # Go to Point
        if self.mode in [Mode.TAKEOFF, Mode.LOCALIZATION, Mode.HOME]:
            if len(self.waypoint.e) != 1:    # wait until Navigator updates
                return
            p = np.array([self.waypoint.e[0],self.waypoint.n[0]])
            des_alt = self.waypoint.alt[-1]

            cmd_vel = pointController(p,self.pos,self.vel)
            cmd_vel_alt = pointController(des_alt, self.alt, self.vel_alt)

            self.cmd_vel.x = cmd_vel[0]
            self.cmd_vel.y = cmd_vel[1]
            self.cmd_vel.z = cmd_vel_alt

        # Follow path
        elif self.mode == Mode.SEARCH:
            if len(self.waypoint.e) != 2:    # wait until Navigator updates
                return
            p1 = np.array([self.waypoint.e[0], self.waypoint.n[0]])
            p2 = np.array([self.waypoint.e[1], self.waypoint.n[1]])
            des_alt = self.waypoint.alt[-1]

            cmd_vel, cmd_yaw = pathController(p1,p2,self.pos,self.vel)
            cmd_vel_alt = pointController(des_alt, self.alt, self.vel_alt)

            self.cmd_vel.x = cmd_vel[0]
            self.cmd_vel.y = cmd_vel[1]
            self.cmd_vel.z = cmd_vel_alt
            self.cmd_yaw = cmd_yaw

        # Landing Controller
        elif self.mode == Mode.LANDING:
            current_yaw = self.attitude[2]
            if self.tag_point_x:
                target_x_enu = self.tag_point_x * math.cos(current_yaw) - self.tag_point_y * math.sin(current_yaw)
                target_y_enu = self.tag_point_x * math.sin(current_yaw) + self.tag_point_y * math.sin(current_yaw)
                p = np.array([target_x_enu, target_y_enu])
                des_alt = 3;
                
                
                cmd_vel = pointController(p,self.pos,self.vel)
                cmd_vel_alt = pointController(des_alt, self.alt, self.vel_alt)
                print("yaw",current_yaw, "p", p, "pos", self.pos, "vel", self.vel, "cmd_vel", cmd_vel)
                self.cmd_vel.x = cmd_vel[0]
                self.cmd_vel.y = cmd_vel[1]
                self.cmd_vel.z = cmd_vel_alt
                self.cmd_yaw = self.tag_point_yaw
            #else:
            #    p = np.array([self.waypoint.e[0], self.waypoint.n[0]])
            #    des_alt = self.waypoint.alt[-1]
            #    cmd_vel = pointController(p, self.pose,self.vel)
            #    cmd_vel_alt = pointController(des_alt, self.alt, self.vel_alt)
            #    self.cmd_vel.x = cmd_vel[0]
            #    self.cmd_vel.y = cmd_vel[1]
            #    self.cmd_vel.z = cmd_vel_alt
            #    self.cmd_yaw = current_yaw


    ## Process Functions
    def publish(self):
        """ publish waypoints and navigation messages """
        self.cmd.header.stamp = rospy.get_rostime()
        self.cmd.velocity = self.cmd_vel
        self.cmd.yaw = self.cmd_yaw
        self.cmd_pub.publish(self.cmd)

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.controlLoop()
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    controller = Controller()
    controller.run()
