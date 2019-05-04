import numpy as np
import rospy


"""
TODO: Get height from pixhawk
    May need another subscriber -> store height as its published
"""


def getUncertainty(h):
    sigma = 2 + h/50.
    return sigma


class BeaconLocalization():

    def __init__(self):
        self.beacon_locations = {}      # stores beacon position and uncertainty {id : [(x,y),sigma]}

        ## Init Node
        #rospy.init_node('BeaconLocalizer', anonymous=True)

        # Subscribers
        #rospy.Subscriber('/beacon/position', Pose2D, self.beaconCallback, queue_size=10)
        #rospy.Subscriber('/drone/position', Pose, self.poseCallback, queue_size=10)

        # Publishers
        #self.obj_pub = rospy.Publisher('/FilteredBeaconLocations', Pose2D, queue_size=10)


    def beaconCallback(self,msg):
        sigma = getUncertainty(self.h)
        for beacon in msg:
            if beacon.id in self.beacon_locations:
                self.beacon_locations[beacon.id] = updateBeaconLocation(beacon.pos,sigma)
            else:
                self.beacon_locations[beacon.id] = [beacon.pos, sigma]


    def poseCallback(self,msg):
        self.h = msg.position[2]


    def updateBeaconLocation(self,x,sigma):
        pass



    def publish(self):
        # TODO: publish beacon_locations
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

