#!/usr/bin/env python

"""
Controller Script for Autonomous Mission
"""

import rospy
import numpy.linalg as npl
from modeController import Mode

# Import message types
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped, Point, Vector3, TwistStamped
from mavros_msgs.msg import PositionTarget
from aa241x_mission.msg import MissionState
from aa241x_commander.msg import Waypoint



V_MAX = 5.0     # in m/s

def pointController(p_des,p_cur,v_cur):
    kp = 1.0
    kd = 0.1
    delta_p = p_des-p_cur
    v_cmd = kp*delta_p - kd*v_cur

    if npl.norm(v_cmd) > V_MAX:
        v_cmd = v_cmd/npl.norm(v_cmd) * V_MAX

    return v_cmd

def pathControler(p1,p2,p_cur,v_cur):
    direction_vector = np.array([p2[0]-p1[0], p2[1]-p1[1], 0.0])
    direction = direction_vector / np.linalg.norm(direction_vector)
    if direction[2] == 0:
        coeff_c = 0
    else:
        coeff_c = 1
    coeff_a = 1
    coeff_b = -(direction_vector[0] + direction_vector[2]) / direction_vector[1]
    coeff_d = - coeff_a*p1[0] - coeff_b*p1[1]

    error = - (coeff_a*p_cur[0] + coeff_b*p_cur[1] + coeff_c*p_cur[2] + coeff_d) / math.sqrt(coeff_a**2 + coeff_b**2 + coeff_c**2)
    normal_vector = np.array([coeff_a, coeff_b, coeff_c])
    normal_vector = normal_vector / math.sqrt(coeff_a**2 + coeff_b**2 + coeff_c**2)
    direction_vector = direction

    velocity = v_cur
    error_dot = np.inner(velocity, normal_vector)

    k_e = 1
    k_e_dot = 0.1
    g = 0.1     # v_cmd = g*v_par + v_perp

    normal_output = k_e * error + k_e_dot * error_dot
    direction_output = V_MAX

    output = g*direction_vector*direction_output + normal_vector*normal_output
    output = output/npl.norm(output) * V_MAX
    return output


class Controller():
    def __init__(self):
        rospy.init_node('controller', anonymous=True)

        # class variables
        self.mode = None
        self.pos = None     # current drone position
        self.vel = None
        self.prev_alt = 0   # if no altitude given, maintain previous

        self.e_offset = 0
        self.n_offset = 0
        self.u_offset = 0

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
        rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, self.velCallback)
        rospy.Subscriber('/mission_state',MissionState,self.missionStateCallback)

    ## Callbacks
    def modeCallback(self,msg):
        self.mode = Mode(msg.data)

    def poseCallback(self,msg):
        pos = msg.pose.position
        self.pos = np.array([pos.x+self.e_offset, pos.y+self.n_offset])
        self.alt = pos.z+ self.u_offset

    def velCallback(self,msg):
        vel = msg.twist.linear
        self.vel = np.array([vel.y,vel.x])

    def waypointCallback(self,msg):
        if len(msg.alt) > 0:
            self.waypoint = msg
            self.prev_alt = msg.alt[-1]

    def missionStateCallback(self,msg):
        self.e_offset = msg.e_offset
        self.n_offset = msg.n_offset
        self.u_offset = msg.u_offset


    ## Main Loop for Navigator
    def controlLoop(self):

        # Specialized Controllers
        if self.mode == Mode.IDLE:  # do nothing
            self.cmd_vel.x = 0
            self.cmd_vel.y = 0
            self.cmd_pos.z = self.prev_alt
        elif self.mode == Mode.TAKEOFF:
            p = np.array([self.waypoint.e,self.waypoint.n])
            cmd_vel = pointController(p,self.pos,self.vel)
            self.cmd_vel.x = cmd_vel[0]
            self.cmd_vel.y = cmd_vel[1]
            self.cmd_pos.z = self.waypoint.alt
        elif self.mode == Mode.SEARCH:
            p1 = np.array([self.waypoint.e[0], self.waypoint.n[0]])
            p2 = np.array([self.waypoint.e[1], self.waypoint.n[1]])
            cmd_vel = pathControler(p1,p2,self.pos,self.vel)
            self.cmd_vel.x = cmd_vel[0]
            self.cmd_vel.y = cmd_vel[1]
            self.cmd_pos.z = self.waypoint.alt
        elif self.mode == Mode.LOCALIZATION:
            p = np.array([self.waypoint.e,self.waypoint.n])
            cmd_vel = pointController(p,self.pos,self.vel)
            self.cmd_vel.x = cmd_vel[0]
            self.cmd_vel.y = cmd_vel[1]
            self.cmd_pos.z = self.waypoint.alt
        elif self.mode == Mode.HOME:
            p = np.array([self.waypoint.e,self.waypoint.n])
            cmd_vel = pointController(p,self.pos,self.vel)
            self.cmd_vel.x = cmd_vel[0]
            self.cmd_vel.y = cmd_vel[1]
            self.cmd_pos.z = self.waypoint.alt
        elif self.mode == Mode.LANDING:
            cmd_vel = pointController(p,self.pos,self.vel)
            self.cmd_vel.x = cmd_vel[0]
            self.cmd_vel.y = cmd_vel[1]
            self.cmd_pos.z = 0

    ## Process Functions
    def publish(self):
        """ publish waypoints and navigation messages """
        self.cmd.header.stamp = rospy.Time.now();
        self.cmd.position = self.cmd_pos;
        self.cmd.velocity = self.cmd_vel;
        self.cmd_pub.publish(self.cmd);

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.controlLoop()
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    controller = Controller()
    controller.run()
