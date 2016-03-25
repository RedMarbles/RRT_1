"""
Defines the map class and functions
"""

import numpy as np
from matplotlib import pyplot as plt

class Map:
	"""Represents the full state and interface of the map"""

	map_limits = {	'xmin':-5, 
					'xmax':+5, 
					'ymin':-5, 
					'ymax':+5  }	# Limits of the map
	default_params = { 	'bot_length':0.5, 
						'bot_color':'b',
						'path_color':'b' }

	def __init__(self):
		""" Map class Constructor """
		#TODO:
		# Choose to update default values for the following:
		# * map_limits

	def addRectObstacle(self, ...):
		""" Adds a rectangular obstacle to the map """
		#TODO

	def inputObstacles(self, ...):
		""" Takes input from the user to draw obstacles """
		#TODO

	def drawTreeMap(self, states_tree, draw_params=None):
		""" Accepts tree of states, and outputs the map with obstacles and the accepted states 
		    states_tree : Tree object 
		    draw_params : Dictionary that holds the """
		# Merge the default parameters with the argument
		params = default_params.copy()
		if(draw_params!=None):
			params.update(draw_params)
		##

		#TODO : compelte this function

	def drawStatesMap(self, states_list, draw_params=None):
		#TODO
