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
	conf_arr = bl.calcReliability(sigma_arr)

	print 'At Height {:.1f} m'.format(h)
	print '90% Condidence at {:d} measurements'.format(np.arange(1,N+1)[conf_arr>0.9][0])
	print '95% Condidence at {:d} measurements'.format(np.arange(1,N+1)[conf_arr>0.95][0])

	fig,axes = plt.subplots(2,1,sharex=True)
	axes[0].plot(range(N),sigma_arr)
	axes[0].grid()
	axes[0].set_ylabel('sigma')
	axes[0].set_title('Filter Performance')
	axes[1].plot(range(N),conf_arr)
	axes[1].grid()
	axes[1].set_ylabel('confidence')
	axes[1].set_xlabel('n measurements')

def testUncertainty():
	h_arr = [30,40,50,60,70,80,90,100]
	num_arr = []
	an_arr = []
	for h in h_arr:
		std = bl.getUncertainty(h)
		loc = np.array([0,0])

		# numerically determine pass rate
		err_arr = []
		N = 10000
		for i in range(N):
			meas = loc + np.random.normal(loc=0,scale=std,size=2)
			err = npl.norm(meas-loc)
			err_arr.append(err)
		err_arr = np.array(err_arr)
		num_pass_rate = np.sum(err_arr<1.0)/float(N)
		an_pass_rate = bl.calcReliability(std)
		num_arr.append(num_pass_rate)
		an_arr.append(an_pass_rate)

	plt.figure()
	plt.plot(h_arr,num_arr)
	plt.plot(h_arr,an_arr)
	plt.xlabel('h')
	plt.ylabel('Pass Rate')
	plt.legend(['numerical', 'analytical'])
	plt.grid()


def main():
	testKalmanFilter()
	testUncertainty()

	plt.show()

if __name__ == '__main__':
	main()

