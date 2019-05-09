import numpy as np
import rospy

from aa241x_mission.msg import SensorMeasurement, PersonEstimate
from geometry_msgs.msg import Pose, PoseStamped


"""
TODO: Need to define BeaconsLocalization message    (see Zach)

"""


def getUncertainty(h):
    sigma = 2 + h/50.
    return sigma


class BeaconLocalization():

    def __init__(self):
        self.beacon_locations = {}      # stores beacon position and uncertainty {id : [(x,y),sigma]}

        ## Init Node
        rospy.init_node('BeaconLocalizer', anonymous=True)

        # Subscribers
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.poseCallback)
        rospy.Subscriber("/measurement", SensorMeasurement, self.beaconCallback);

        # Publishers
        # self.obj_pub = rospy.Publisher('/localizer/localized_beacons', LocalizedBeacons, queue_size=10)
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
                self.beacon_locations[id_num] = updateBeaconLocation(np.array([n,e]),sigma)
            else:
                self.beacon_locations[id_num] = [np.array([n,e]), sigma]

    def poseCallback(self,msg):
        self.h = msg.pose.position.z


    def updateBeaconLocation(self,x,sigma):
        # TODO: Implement Kalman Filter
        pass



    def publish(self):
        # TODO: publish beacon_locations
        # Publish all beacons to person estimate
        # Only publish beacons locations we are satisfied to localized_beacons
        #       localized_beacons will b used by the mode_controller to determine when to swtich to localize mode,
        #       and will be used by the navigator to determine when localization is done
        pass


    def run(self):
        """ main loop """
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()


if __name__ == "__main__":
    localizer = BeaconLocalization()
    localizer.run()

