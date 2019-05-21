# Contains the search path in lake lag frame ( in meters at the moment, may want to convert to lat lon )
# TODO: Optimize search path


search_path = [
[0,0],	# start at center

[30,0],
[30,30],
[-30,30],
[-30,-30],

[60,-30],
[60,60],
[-60,60],
[-60,-60],

[90,-60],
[90,90],
[-90,90],
[-90,-90],

[120,-90],
[120,120],
[-120,120],
[-120,-120],

[150,-120],
[150,150],
[-150,150],
[-150,-150],
[-150,150]

]
