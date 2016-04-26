"""
Defines the map class and functions
This implementation of the map class uses a gridworld representation
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
		self.map_grid = self.loadBitmap("Maps/Map1.bmp")

	def addRectObstacle(self, ...):
		""" Adds a rectangular obstacle to the map """
		pass #TODO

	def inputObstacles(self, ...):
		""" Takes input from the user to draw obstacles """
		pass #TODO

	def drawTreeMap(self, states_tree, draw_params=None):
		""" Accepts tree of states, and outputs the map with obstacles and the accepted states 
		    states_tree : Tree object 
		    draw_params : Dictionary that holds the parameters of the draw function"""
		# Merge the default parameters with the argument
		params = Map.default_params.copy()
		if(draw_params!=None):
			params.update(draw_params)
		##

		#TODO : complete this function

	def loadBitmap(self, filename="Maps/Map1.bmp"):
		""" Loads a RGB bitmap to use as a map. Only the first channel is looked at.
		"""
		# Open file and read all data
		with open(filename,"rb") as file:
			data = bytearray(file.read())

		# Parse bitmap header

		# Verify that it is a bitmap
		if not( data[0]==66 AND data[1]==77 ):
			print("File is not a bitmap")
			return 0
		
		def little_endian(data, location, size):
			""" Extracts 'size' bytes of data in little-endian order at 'location' """
			output = 0
			for i in range(size):
				output = output | data[location+i]<<(8*i)
			return output

		bitmap_size = little_endian(data, 2, 4)
		start_offset = little_endian(data, 10, 4)
		header_size = little_endian(data, 14, 4)
		if (header_size<40):
			print("Unknown format")
			return 0
		width_pixels = little_endian(data, 18, 4)
		#TODO : Complete with information from https://en.wikipedia.org/wiki/BMP_file_format

	def drawStatesMap(self, states_list, draw_params=None):
		""" Draw only a specified list of states in order, to mark a specific path """
		pass #TODO : complete this function

	def 
