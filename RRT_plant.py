"""
Defines the classes required for the basic simulation.
* State - Represents a fixed state of the system (x,y,theta,time)
* ControlInput - The input control vector used to drive the vehicle. Currently uses a (v,omega)
	control for a (2,0) differential drive robot, but later needs to be changed to the (v,beta)
	control for a (1,1) steerable robot.
* Model - Holds properties of the robot, such as maximum velocities and turning radii
* Plant - Runs the simulation, such as updating robot positions
* Node - Forms the basic nodes of the RRT tree, and contains information about the parent
	and child nodes, as well as state and control to reach state
* Tree - A tree structure for the RRT
"""

import numpy as np
from matplotlib import pyplot as plt

model = { 'v_min': -5, # [m/s]
		  'v_max': +5, # [m/s]
		  'omega_min': -5, # [radians/s]
		  'omega_max': +5, # [radians/s]
		  'min_turn_radius': 3, # [meters]
		  'dt': 0.3 #[seconds]
		  }
model['dx_max']  = model['v_max'] * model['dt'] # maximum positive movement in single timestep
model['dth_max'] = model['omega_max'] * model['dt'] # maximum positive rotation in a single timestep
model['dx_min']  = model['v_min'] * model['dt'] # maximum negative movement in single timestep
model['dth_min'] = model['omega_min'] * model['dt'] # maximum negative rotation in a single timestep


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

	def __eq__(self, other):
		if ( (self.x==other.x) and (self.y==other.y) and (self.theta==other.theta) ):
			return true
		else:
			return false

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

	def saturate(self, model):
		""" Return a saturated control input """
		v = self.v
		omega = self.omega
		v = np.min( [ v, model['v_max'] ] )
		v = np.max( [ v, model['v_min'] ] )
		omega = np.min( [ omega, model['omega_max'] ] )
		omega = np.max( [ omega, model['omega_min'] ] )
		return ControlInput(v, omega)



class Plant:
	"""Represents the physical system itself, the simulation of the system"""

	def __init__(self):
		"""Constructor"""
		self.tree = Tree()


	def update(self, X=State(), U=ControlInput()):
		"""Apply control on the state"""
		U = U.saturate(self.v_lim, -self.v_lim, self.omega_lim, -self.omega_lim)
		(v,w) = (U.v,U.w)

		d_theta = w * self.dt
		x_new = X.x + v*np.cos(X.theta + d_theta/2)*self.dt
		y_new = X.y + v*np.sin(X.theta + d_theta/2)*self.dt
		theta_new = X.theta + d_theta
		return State(x_new, y_new, theta_new, X.time+self.dt)



class Node:
	"""Represents one node on the tree"""

	def __init__(self, state=State(), parent=0, control=ControlInput()):
		self.state = state      # Current state of the robot
		self.parent = parent        # Address of previous state of the robot
		self.control = control  # Control used to reach current state from root
		self.children = []      # List of all children of this node



class Tree:
	"""Represents the tree structure"""

	def __init__(self, value=None):
		"""Constructor
		    value : The initial root node of the tree 
		"""
		self.data = []
		if(isinstance(value, Node)): 
			#If a node is given, set it as the new root
			value.parent = -1
			self.data.append(value)
		else if(isinstance(value, Tree)):
			#If a tree is given, copy it into the blank data structure
			self.data.extend(Tree.data)
		else if(isinstance(value, State)):
			#If a state is given, create a new node and set it as the root
			self.data.append( Node(state=value, parent=-1, control=ControlInput()) )

	def __getitem__(self, key):
		#Note that the key here should be an integer
		return self.data[key]

	def __setitem__(self, key, value):
		#Note that the key here should be an integer
		if (isinstance(value, Node)):
			self.data[key] = value
		else print("Error: not a node type")

	def append(self, state, control, parent):
		if (isinstance(state, State) and isinstance(control,ControlInput) and isinstance(parent,int)):
			#Add address of child in the parent node
			self.data[parent].children.append( len(self.data) )
			#Create node with state and control and add it 
			self.data.append( Node(state=state, control=control, parent=parent)  )
		else printf("Error: Invalid data types")

	def __len__(self):
		return len(self.data)

