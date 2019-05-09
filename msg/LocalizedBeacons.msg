# A list of localized beacons and their localization status

# the number of measurements (N) contained in this message
int32 num_measurements

# list of ids of the localized beacons
int32[] ids

# list of the north and east coordinates of the localized beacons [m]
float32[] n
float32[] e

# list of uncertainties (standard deviation) of the localized beacons [m]
float32[] sigma
