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
from RRT_plant import Plant
from RRT_plant import Node
from RRT_plant import Tree

from RRT_map import Map



class Planner:
	""" Do stuff """
	def __init__(self, initial_state=State(), final_state=State()):
		"""Constructor
		   initial_state : The root node of the tree
		   final_state : Either the goal state, or else a list of goal states
		"""
		self.model = model  # The model parameters
		self.plant = Plant()  # The simulation environment
		self.tree = Tree(initial_state)  # Initialize the tree
		self.goals = [final_state]       # The goal to aim for
		self.map = Map()   # The obstacle map of the environment
		self.current_target = State()  # Dummy initialization to mark the next point to go towards

	def selectNextTarget():
		""" Sets current_target to the next point to aim for """
		# Simple version - alternate between goal and random point
		if(self.current_target == self.goals[0]):
			collision = True
			while(collision):
				new_x = np.random.uniform(map.map_limits['xmin'], map.map_limits['xmax'])
				new_y = np.random.uniform(map.map_limits['ymin'], map.map_limits['ymax'])
				new_theta = np.random.uniform( -np.pi, +np.pi )
				collision = map.checkCollision(new_x, new_y)

			self.current_target = State(new_x, new_y, new_theta)
		else:
			self.current_target = self.goals[0]


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