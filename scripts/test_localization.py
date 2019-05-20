import numpy as np
import numpy.linalg as npl
import matplotlib.pyplot as plt
import scipy.stats as sps

import beaconLocalization as bl


def testKalmanFilter():
	h = 30
	beacon_location = np.array([0,0])
	localizer = bl.BeaconLocalization(debug=True)

	mu_arr = []
	sigma_arr = []

	N = 60	# simulate N measurement
	for i in range(N):

		# simulate measurement
		std = bl.getUncertainty(h)
		meas = beacon_location + np.random.normal(loc=0,scale=std,size=2)

		if i == 0:
			localizer.beacon_locations[0] = [meas,std]
		else:
			localizer.beacon_locations[0] = localizer.updateBeaconLocation(0,meas,std)

		# store values
		mu_arr.append(localizer.beacon_locations[0][0])
		sigma_arr.append(localizer.beacon_locations[0][1])

	mu_arr = np.array(mu_arr)
	sigma_arr = np.array(sigma_arr)

	plt.plot(range(N),sigma_arr)
	plt.grid()
	plt.xlabel('n measurements')
	plt.ylabel('sigma')
	plt.title('sigma vs n')

def testUncertainty():
	h = 30
	std = bl.getUncertainty(h)
	loc = np.array([0,0])

	# numerically determine pass rate
	err_arr = []
	N = 100000
	for i in range(N):
		meas = loc + np.random.normal(loc=0,scale=std,size=2)
		err = npl.norm(meas-loc)
		err_arr.append(err)
	err_arr = np.array(err_arr)
	num_pass_rate = np.sum(err_arr<1.0)/float(N)
	print "Numeric Pass Rate: ", num_pass_rate

	# analytically determine pass rate
	an_pass_rate = bl.calcReliability(std)
	print "Analytic Pass Rate: ", an_pass_rate


def main():
	testKalmanFilter()
	testUncertainty()

	plt.show()

if __name__ == '__main__':
	main()

