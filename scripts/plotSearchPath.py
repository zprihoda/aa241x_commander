import searchPath
import pylab as pl

if __name__ == '__main__':
	wp_list = searchPath.search_path
	wp = pl.array(wp_list)
	pl.plot(wp[:,0],wp[:,1])
	pl.xlabel('e (m)')
	pl.ylabel('n (m)')
	pl.show()
