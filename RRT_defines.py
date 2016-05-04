"""
Defines the State and ControlInput classes, to represent the X and U vectors
"""

import numpy as np
from matplotlib import pyplot as plt

class State:
	"""Represents the state X of the system"""

	def __init__(self, x=0.0, y=0.0, theta=0.0, time=0.0):
		"""Constructor."""
		self.x = x
		self.y = y
		self.theta = theta
		self.time = time
		self.__angle_check()

	def __angle_check(self):
		"""Make sure angle is between -pi and +pi"""
		while (self.theta > +np.pi): self.theta -= 2*np.pi
		while (self.theta < -np.pi): self.theta += 2*np.pi

	def __repr__(self):
		return "[x:%f, y:%f, theta:%f]" % (self.x, self.y, self.theta)

	def __add__(self, other):
		return State(self.x+other.x, self.y+other.y, self.theta+other.theta)

	def __sub__(self, other):
		return State(self.x-other.x, self.y-other.y, self.theta-other.theta)

	def rho(self):
		"""Calculate the eucledian distance between this (x,y) point and the origin"""
		return np.sqrt(self.x*self.x + self.y*self.y)


class ControlInput:
	"""Represents the input control vector U of the system"""

	def __init__(self, v=0, omega=0):
		"""Constructor"""
		self.v = v
		self.omega = omega

	def __repr__(self):
		return "[v:%f, omega:%f]" % (self.v, self.omega)

	def saturate(self, v_max=5, v_min=-5, omega_max=5, omega_min=-5):
		""" Return a saturated control input """
		v = self.v
		omega = self.omega
		v = np.min( [v, v_max] )
		v = np.max( [v, v_min] )
		omega = np.min( [omega, omega_max] )
		omega = np.max( [omega, omega_min] )
		return ControlInput(v, omega)


class Plant:
	"""Represents the physical system itself, the simulation of the system"""

	def __init__(self, v_lim=5, omega_lim=5):
		"""Constructor"""
		self.v_lim = v_lim # [m/s]
		self.omega_lim = omega_lim # [radians/sec]
		self.min_turn_radius = 3 # [meters]
		self.dt = 0.3 # [seconds]
		self.dx_max = self.v_lim * self.dt # maximum movement
		self.dth_max = self.omega_lim * self.dt # maximum rotation

	def update(self, X=State(), U=ControlInput()):
		"""Apply control on the state"""
		U = U.saturate(self.v_lim, -self.v_lim, self.omega_lim, -self.omega_lim)
		(v,w) = (U.v,U.w)

		d_theta = w * self.dt
		x_new = X.x + v*np.cos(X.theta + d_theta/2)*self.dt
		y_new = X.y + v*np.sin(X.theta + d_theta/2)*self.dt
		theta_new = X.theta + d_theta
		return State(x_new, y_new, theta_new, X.time+self.dt)

	def distance(self, X1, X2):
		"""Return distance between two states"""
		return self.distance_euclidean(X1, X2)

	def distance_euclidean(self, X1, X2):
		"""Calculate simple eucledian distance"""
		dX = X2-X1
		d = np.sqrt(dX.x*dX.x + dX.y*dX.y)
		return d

	def distance_geometric_sans_parking(self, X1, X2):
		"""
		Calculate distance between states using formula:
		distance = \sqrt{(x_2-x_1)^2 + (y_2-y_1)^2} + 2*r_{min}*\phi \\
		\phi = \mathrm{atan2} \left( \frac{y_2-y_1}{x_2-x_1} \right)
		"""
		dX = X2-X1
		phi = np.arctan2(dX.y,dX.x)
		d = np.sqrt(dX.x*dX.x + dX.y*dX.y) + 2*self.min_turn_radius*np.abs(phi - X1.theta)
		return d

	def extend(self, X, X_rand):
		"""Calculates next step from X to X_rand"""
		return self.extend_simple(X, X_rand)

	def extend_simple(self, X, X_rand):
		"""The extend function, for a simple holonomic system"""
		dX = X_rand - X
		dist = dX.rho()

		# If distance is less than unit distance, return directly
		if (dist<=dx_max) : return State(X_rand.x, X_rand.y)

		#Normalizing the difference vector to get unit vector
		#Also multiplying by dx_max to get RRT unit jump
		x_new = dX.x / dist * dx_max
		y_new = dX.y / dist * dx_max
		return State(x_new, y_new)


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
		elif(isinstance(value, Tree)): 
			self.data.extend(Tree.data)
		elif(isinstance(value, State)):
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

