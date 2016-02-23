"""
Defines the State and Control classes, to represent the X and U vectors
"""

import numpy as np
from matplotlib import pyplot as plt

class State:
	"""Represents the state X of the system"""

	def __init__(self, x=0.0, y=0.0, theta=0.0):
		"""Constructor."""
		self.x = x
		self.y = y
		self.theta = theta
		self.__angle_check()

	def __angle_check(self):
		"""Make sure angle is between 0 and 2*pi"""
		while (self.theta>2*np.pi): self.theta -= np.pi
		while (self.theta<0): self.theta += np.pi

	def __repr__(self):
		return "[x:%f, y:%f, theta:%f]" % (self.x, self.y, self.theta)

	def __add__(self, other):
		return State(self.x+other.x, self.y+other.y, self.theta+other.theta)

	def __sub__(self, other):
		return State(self.x-other.x, self.y-other.y, self.theta-other.theta)

class Control:
	"""Represents the control vector U of the system"""

	def __init__(self, v=0, omega=0):
		"""Constructor"""
		self.v = v
		self.omega = omega

	def __repr__(self):
		return "[v:%f, omega:%f]" % (self.v, self.omega)


class Plant:
	"""Represents the physical system itself"""

	def __init__(self, v_lim=5, omega_lim=0.2):
		"""Constructor"""
		self.v_lim = v_lim # [m/s]
		self.omega_lim = omega_lim # [radians/sec]
		self.min_turn_radius = 3 # [meters]
		self.dt = 0.3 # [seconds]
		self.dx_max = self.v_lim * self.dt # maximum movement
		self.dth_max = self.omega_lim * self.dt # maximum rotation

	def update(self, X=State(), U=control()):
		"""Apply control on the state"""
		(v,w) = (U.v,U.w)
		v = np.min( [v, self.v_lim] )
		v = np.max( [v,-self.v_lim] )
		w = np.min( [w, self.omega_lim] )
		w = np.max( [w,-self.omega_lim] )
		d_theta = w * self.dt
		x_new = X.x + v*np.cos(X.theta + d_theta/2)
		y_new = X.y + v*np.sin(X.theta + d_theta/2)
		theta_new = X.theta + d_theta
		return State(x_new, y_new, theta_new)

	def distance(self, X1, X2):
		"""Calculate distance between states"""
		dX = X2-X1
		d = np.sqrt(dX.x*dX.x + dX.y*dX.y) + 2*self.min_turn_radius*np.abs(dX.theta)

	def extend(self, X, X_rand):
		"""Calculates next step from X to X_rand"""
		dX = X_rand - X
		dist = np.sqrt(dX.x*dX.x + dX.y*dX.y)
		dx = dX.x / dist * self.dx_max
		dy = dX.y / dist * self.dx_max
		#TODO


class Node:
	"""Represents one node on the tree"""

	def __init__(self, state=State(), root=0):
		self.state = state
		self.root = root

class Tree:
	"""Represents the tree structure"""

	def __init__(self, value=None):
		"""Constructor"""
		self.data = []
		if(isinstance(value, Node)): 
			self.data.append(value)
		else if(isinstance(value, Tree)): 
			self.data.extend(Tree.data)
		else if(isinstance(value, State)):
			self.data.append( Node(state=value, root=-1) )

	def __getitem__(self, key):
		return self.data[key]

	def __setitem__(self, key, value):
		if (isinstance(value, Node)):
			self.data[key] = value
		else print("Error: not a node type")

	def append(self, value):
		if (isinstance(value, Node)):
			self.data.append(value)
		else printf("Error: not a node type")

	def __len__(self):
		return len(self.data)

