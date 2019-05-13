import numpy as np
import matplotlib.pyplot as plt

from Controller import pathController, pointController


def testPathController():
	p1 = np.array([0,0])
	p2 = np.array([100,100])
	p = np.array([0,-50])
	v = 0
	dt = 0.1

	p_arr = [p]
	for i in range(500):
		v = pathController(p1,p2,p,v)
		p = p + dt*v
		p_arr.append(p)
	p_arr = np.array(p_arr)

	fig,ax = plt.subplots(1,1)
	plt.plot(p_arr[:,0],p_arr[:,1])	
	plt.plot([p1[0],p2[0]], [p1[1],p2[1]],'--k')
	ax.set_aspect('equal')
	plt.show()

def testPointController():
	p_des = np.array([10,10])
	p = np.array([5,0])
	v = np.array([0,0])
	dt = 0.1

	p_arr = [p]
	for i in range(100):
		v = pointController(p_des,p,v)
		p = p + dt*v
		p_arr.append(p)
	p_arr = np.array(p_arr)

	fig,axes = plt.subplots(3,1)
	axes[0].plot(p_arr[:,0],p_arr[:,1])	
	axes[1].plot(p_arr[:,0])
	axes[2].plot(p_arr[:,1])
	plt.show()


def main():
	testPathController()
	# testPointController()

if __name__ == "__main__":
	main()