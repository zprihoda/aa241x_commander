#!/usr/bin/env python

"""
Beacon Localization Script for Autonomous Mission
"""

import numpy as np
import numpy.linalg as npl
import scipy.stats as sps
import rospy

from aa241x_mission.msg import SensorMeasurement, PersonEstimate
from aa241x_commander.msg import LocalizedBeacons
from geometry_msgs.msg import Pose, PoseStamped


CERTAINTY_THRESHOLD = 0.90    # publish localized beacon once we're 90% sure of its location within 1 meter
POS_DESIRED = 1               # position accuracy corresponding to certainty threshold
Z_THRESH = sps.norm.ppf((1+CERTAINTY_THRESHOLD)/2)  # convert certainty to sigma (95%->~2 sigma)


def getUncertainty(h):
    sigma = 2 + h/50.
    return sigma


class BeaconLocalization():

    def __init__(self):
        self.beacon_locations = {}      # stores beacon position and uncertainty {id : [(x,y),sigma]}
        self.h = None

        ## Init Node
        rospy.init_node('BeaconLocalizer', anonymous=True)

        # Subscribers
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallback)
        rospy.Subscriber("/measurement", SensorMeasurement, self.beaconCallback);

        # Publishers
        self.lbeac_pub = rospy.Publisher('/localizer/localized_beacons', LocalizedBeacons, queue_size=10)
        self.person_pub = rospy.Publisher('/person_found',PersonEstimate,queue_size=10)

    def beaconCallback(self,msg):
        sigma = getUncertainty(self.h)

        for i in range(msg.num_measurements):
            # Load Data
            id_num = msg.id[i]
            n = msg.n[i]
            e = msg.e[i]

            # Update/Store Estimates
            if id_num in self.beacon_locations.keys():
                self.beacon_locations[id_num] = updateBeaconLocation(id_num,np.array([n,e]),sigma)
            else:
                self.beacon_locations[id_num] = [np.array([n,e]), sigma]

    def poseCallback(self,msg):
        self.h = msg.pose.position.z

    def updateBeaconLocation(self,id_num,y_k,sigma):
        """ Kalman Filter with 0 dynamics """
        r_k = np.identity(2) * (sigma) ** 2

        prior_mean,prior_sigma = self.beacon_locations[id_num]
        prior_covariance = np.identity(2) * post_sigma**2
        sigma = np.identity(2)*sigma

        K = np.matmul(prior_cov, np.linalg.inv(r_k + prior_cov))
        new_mean = prior_mean + np.matmul(K, y_k - prior_mean)
        new_cov = prior_cov - np.matmul(K, prior_cov)
        new_sigma = np.sqrt(new_cov[0,0])
        return new_mean, new_sigma

    def publish(self):

        lbeac = LocalizedBeacons()
        lbeac.ids = []
        lbeac.n = []
        lbeac.e = []
        lbeac.sigma = []

        for beacon_id in self.beacon_locations.keys():
            pos,sigma = self.beacon_locations[beacon_id]

            # Publish to person estimate
            msg = PersonEstimate()
            msg.header.stamp = rospy.Time.now()
            msg.id = beacon_id
            msg.n = pos[0]
            msg.e = pos[1]
            self.person_pub.publish(msg)

            # publish to localized_beacons
            if sigma/POS_DESIRED <= Z_THRESH:
                lbeac.ids.append(beacon_id)
                lbeac.n.append(pos[0])
                lbeac.e.append(pos[1])
                lbeac.sigma.append(sigma)
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
