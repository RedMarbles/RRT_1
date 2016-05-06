"""
Holds the classes for the controller based around the RRT system. These classes are in charge
of calculating the distance, calculating updates. Note that they may use models of the 
system that are not correct, to simulate modelling errors.
* Controller -
"""

import numpy as np
from matplotlib import pyplot as plt

from RRT_plant import model
from RRT_plant import State
from RRT_plant import ControlInput
from RRT_plant import Node
from RRT_plant import Tree

from RRT_map import Map



class Planner:
	""" Handles the entire simulation """
	def __init__(self, initial_state=State(), final_state=State(), mapfile="Maps/Map1.bmp"):
		"""Constructor
		   initial_state : The root node of the tree
		   final_state : Either the goal state, or else a list of goal states
		"""
		self.model = model  # The model parameters
		self.tree = Tree(initial_state)  # Initialize the tree
		self.goals = [final_state]       # The goal to aim for
		self.map = Map(mapfile)   # The obstacle map of the environment
		self.current_target = State()  # Dummy initialization to mark the next point to go towards



	def selectNextTarget(self):
		""" Sets current_target to the next point to aim for """
		# Simple version - alternate between goal and random point
		if(self.current_target == self.goals[0]):
			collision = True
			while(collision):
				new_x = np.random.uniform(self.map.map_limits['xmin'], self.map.map_limits['xmax'])
				new_y = np.random.uniform(self.map.map_limits['ymin'], self.map.map_limits['ymax'])
				new_theta = np.random.uniform( -np.pi, +np.pi )
				collision = self.map.checkCollision(new_x, new_y)

			self.current_target = State(new_x, new_y, new_theta)
		else:
			self.current_target = self.goals[0]
		return self.current_target


	def findClosestNode(self, goal_state):
		""" Finds the node in the tree that is closest to the goal state """
		address = 0
		min_dist = self.distance(self.tree[0].state,goal_state)
		for i in range( len(self.tree) ):
			curr_dist = self.distance(self.tree[i].state, goal_state)
			if( curr_dist < min_dist ):
				min_dist = curr_dist
				address = i
		return address



	def grow(self, goal_state=None):
		""" Grow the tree in the direction of the target state
			Returns whether the final goal state has been reached
		"""
		if (goal_state is None):
			goal_state = self.current_target
		
		# Find closest node in tree
		node_old = self.findClosestNode(goal_state)

		# Grow tree in the direction of the node, use generalized extend algorithm
		reached_final_goal = self.extend(node_old, goal_state)
		return reached_final_goal



	def extend(self, old_node_address, goal_state):
		""" Extends tree in the direction of the selected goat state
			Returns whether the goal state has been reached
			The extend function, for a simple holonomic system
		"""
		current_state = self.tree[old_node_address].state

		dX = goal_state - current_state
		dist = self.distance(goal_state, current_state)

		# If distance is less than unit distance, return directly
		if (dist<=self.model['dx_max']) :
			new_state = goal_state
		else:
			#Normalizing the difference vector to get unit vector
			#Also multiplying by dx_max to get RRT unit jump
			x_new = dX.x / dist * self.model['dx_max'] + current_state.x
			y_new = dX.y / dist * self.model['dx_max'] + current_state.y
			new_state = State(x_new, y_new)

			# Abort current iteration if we extend onto an obstacle
			if(self.map.checkCollision(x_new, y_new)==True):
				return False
		self.tree.append(state=new_state, parent=old_node_address, control=ControlInput())
		if(self.distance(new_state, self.goals[0]) < self.model['dx_max']) :
			return True
		else :
			return False


	def update(self, X=State(), U=ControlInput()):
		"""Evaluate result of applying control on the state"""
		print("Error - %s.update() function has not been implemented" % self.__class__)
		pass


	def calculate_control(self, old_state, goal_state):
		""" Calculate the control needed to reach the new state """
		print("Error - %s.calculate_control() function has not been implemented" % self.__class__)
		pass


	def distance(self, X1, X2):
		"""Calculate simple eucledian distance between two states"""
		dX = X2-X1
		d = np.sqrt(dX.x*dX.x + dX.y*dX.y)
		return d




class PlannerRRTSimple_d1(Planner):
	""" Combines the simple one-node-at-a-time RRT algorithm with the first version of
	    the distance function
	"""

	def __init__(self, initial_state=State(), final_state=State(), mapfile="Maps/Map1.bmp"):
		Planner.__init__(self, initial_state, final_state, mapfile)

	def extend(self, old_node_address, goal_state):
		"""Extends the specified node towards the new node by a single unit"""
		old_node = self.tree[old_node_address]
		old_state = old_node.state
		control = self.calculate_control(old_state, goal_state)
		new_state = self.update(old_state, control)

		# Abort current iteration if we extend onto an obstacle
		if(self.map.checkCollision(new_state.x, new_state.y)==True):
			return False

		self.tree.append(state=new_state, control=control, parent=old_node_address)
		if(self.distance(new_state, self.goals[0]) < self.model['dx_max']):
			return True

	def distance(self, X1, X2):
		"""
		Calculate distance between states using formula:
		distance = \sqrt{(x_2-x_1)^2 + (y_2-y_1)^2} + 2*r_{min}*\phi \\
		\phi = \mathrm{atan2} \left( \frac{y_2-y_1}{x_2-x_1} \right)
		"""
		dX = X2-X1
		phi = np.arctan2(dX.y,dX.x)
		d = np.sqrt(dX.x*dX.x + dX.y*dX.y) + 2*self.model['min_turn_radius']*np.abs(phi - X1.theta)
		return d

	def update(self, X=State(), U=ControlInput()):
		"""Evaluate result of applying control on the state"""
		U = U.saturate(self.model)
		(v,w) = (U.v,U.omega)
		dt = self.model['dt']

		d_theta = w * dt
		x_new = X.x + v*np.cos(X.theta + d_theta/2)*dt
		y_new = X.y + v*np.sin(X.theta + d_theta/2)*dt
		theta_new = X.theta + d_theta
		return State(x_new, y_new, theta_new, X.time+dt)

	def calculate_control(self, old_state, goal_state):
		""" Calculate the control needed to reach the new state """
		dist = self.distance(old_state, goal_state)
		dX = goal_state - old_state
		phi = np.arctan2(dX.y, dX.x)
		dtheta = phi - old_state.theta
		ratio = self.model['dx_max']/dist
		deltax = dX.x * ratio
		deltay = dX.y * ratio
		deltatheta = dtheta

		omega = deltatheta / self.model['dt']
		#v = deltax / deltay * np.tan(old_state.theta + deltatheta/2) / self.model['dt']
		v = np.sqrt( (deltax**2 + deltay**2) / self.model['dt']**2)
		return ControlInput(v,omega)



class PlannerRRTConnect_d1(PlannerRRTSimple_d1):
	""" Combines RRT Connect algorithm with the first version of the distance algoritm """
	def __init__(self, initial_state=State(), final_state=State()):
		PlannerRRTSimple.__init__(self, initial_state, final_state)

	def extend(self, old_node_address, goal_state):
		"""Extend the specified node towards the new node continuously until 
		   it reaches an obstacle or target
		"""
		pass

