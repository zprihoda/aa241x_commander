#!/usr/bin/env python

"""
Beacon Localization Script for Autonomous Mission
"""

import numpy as np
import numpy.linalg as npl
# import scipy.stats as sps
import rospy

from aa241x_mission.msg import SensorMeasurement, PersonEstimate, MissionState
from aa241x_commander.msg import LocalizedBeacons
from geometry_msgs.msg import Pose, PoseStamped


CERTAINTY_THRESHOLD = 0.95    # publish localized beacon once we're 95% sure of its location within 1 meter
POS_DESIRED = 1               # position accuracy corresponding to certainty threshold


def calcReliability(var):
    # pass_rate = sps.chi2.cdf(POS_DESIRED/var,2,loc=0,scale=1)
    pass_rate = var < 0.163     # 0.95 confidence
    # pass_rate = var < 0.217     # 0.90 confidence
    # pass_rate = var < 0.108     # 0.99 confidence

    return pass_rate

def getUncertainty(h):
    sigma = 2 + h/50.
    return sigma


class BeaconLocalization():

    def __init__(self,debug=False):
        self.beacon_locations = {}      # stores beacon position and uncertainty {id : [(x,y),sigma]}
        self.published = {}
        self.h = None
        self.e_offset = 0
        self.n_offset = 0
        self.u_offset = 0

        if not debug:
            ## Init Node
            rospy.init_node('BeaconLocalizer', anonymous=True)

            # Subscribers
            rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallback)
            rospy.Subscriber("/measurement", SensorMeasurement, self.beaconCallback);
            rospy.Subscriber('/mission_state',MissionState,self.missionStateCallback)

            # Publishers
            self.lbeac_pub = rospy.Publisher('/localizer/localized_beacons', LocalizedBeacons, queue_size=10)
            self.person_pub = rospy.Publisher('/person_found',PersonEstimate,queue_size=10)

    def beaconCallback(self,msg):
        sigma = getUncertainty(self.h)

        if self.h is None:
            return

        for i in range(msg.num_measurements):
            # Load Data
            id_num = msg.id[i]
            n = msg.n[i]
            e = msg.e[i]

            # Update/Store Estimates
            if id_num in self.beacon_locations.keys():
                self.beacon_locations[id_num] = self.updateBeaconLocation(id_num,np.array([n,e]),sigma)
            else:
                self.beacon_locations[id_num] = [np.array([n,e]), sigma]
                self.published[id_num] = 0

    def poseCallback(self,msg):
        if self.u_offset == 0:
            return
        self.h = msg.pose.position.z + self.u_offset

    def missionStateCallback(self,msg):
        self.e_offset = msg.e_offset
        self.n_offset = msg.n_offset
        self.u_offset = msg.u_offset


    def updateBeaconLocation(self,id_num,y_k,sigma):
        """ Kalman Filter with 0 dynamics """
        r_k = np.identity(2) * sigma

        prior_mean,prior_sigma = self.beacon_locations[id_num]
        prior_cov = np.identity(2) * prior_sigma

        K = np.matmul(prior_cov, np.linalg.inv(r_k + prior_cov))
        new_mean = prior_mean + np.matmul(K, y_k - prior_mean)
        new_cov = prior_cov - np.matmul(K, prior_cov)
        new_sigma = new_cov[0,0]
        return new_mean, new_sigma

    def publish(self):

        lbeac = LocalizedBeacons()
        lbeac.ids = []
        lbeac.n = []
        lbeac.e = []
        lbeac.sigma = []

        for beacon_id in self.beacon_locations.keys():
            pos,sigma = self.beacon_locations[beacon_id]

            # publish to localized_beacons
            if calcReliability(sigma) >= CERTAINTY_THRESHOLD:
                lbeac.ids.append(beacon_id)
                lbeac.n.append(pos[0])
                lbeac.e.append(pos[1])
                lbeac.sigma.append(sigma)

                # Publish to person estimate once we are 90% positive
                if not self.published[beacon_id]:
                    msg = PersonEstimate()
                    msg.header.stamp = rospy.Time.now()
                    msg.id = beacon_id
                    msg.n = pos[0]
                    msg.e = pos[1]
                    self.person_pub.publish(msg)
                    self.published[beacon_id] = 1

        lbeac.num_beacons = len(lbeac.ids)
        self.lbeac_pub.publish(lbeac)

    def run(self):
        """ main loop """
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()


if __name__ == "__main__":
    localizer = BeaconLocalization()
    localizer.run()
