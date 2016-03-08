"""
Just testing some functions
"""

import numpy as np
from matplotlib import pyplot as plt

X_list = []
U_list = []
dt = 0.4

def state_update(v,omega):
	"Tests the simple delta_X formula"

	X = X_list[ len(X_list) - 1 ] #Get last state
	x = X[0]
	y = X[1]
	theta = X[3]

	if( omega != 0 ):
		dx =  v/omega*( np.sin(theta + omega*dt) - np.sin(theta) )
		dy = -v/omega*( np.cos(theta + omega*dt) - np.cos(theta) )
		dtheta = omega*dt
	else:
		dx = v*np.cos(theta)*dt
		dy = v*np.sin(theta)*dt
		dtheta = 0

	X_list.append( (x+dx, y+dy, theta+dtheta) )
	U_list.append( (v,omega) )

	redraw()
	#end function state_update()


def redraw():
	"Redraws the plots"

	f1 = plt.figure(1)
	#f2 = plt.figure(2)

	for i in range(len(X_list) - 1):
		X1 = X_list[i]
		X2 = X_list[i+1]
		x1 = X1[0]
		y1 = X1[1]
		theta1 = X1[2]
		x2 = X2[0]
		y2 = X2[1]
		theta2 = X2[2]
		f1.plot([x1,x2],[y1,y2],"bo") #plot squares
		f1.plot([x1,x2],[y1,y2],"b") #plot lines
		f1.plot([x1,x1+np.cos(theta1)],[y1,y1+np.sin(theta1),"r"]) #red orientation marker
		f1.plot([x2,x2+np.cos(theta2)],[y2,y2+np.sin(theta2),"r"]) #red orientation marker

	f1.show()
	#end function redraw()


def init(x0, y0, theta0):
	"Reinitialize state"

	X_list = [(x0,y0,theta0)]
	U_list = [(0,0)]


if __name__ == '__main__':
	init(0,0,0)
	state_update(2,0)
	state_update(1,1)
	state_update(2,4)