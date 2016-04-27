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

	def addRectObstacle(self, etc):
		""" Adds a rectangular obstacle to the map """
		pass #TODO

	def inputObstacles(self, etc):
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
			Data format is taken from https://en.wikipedia.org/wiki/BMP_file_format
		"""
		# Open file and read all data
		with open(filename,"rb") as file:
			data = bytearray(file.read())

		# Parse bitmap header

		# Verify that it is a bitmap
		if not( data[0]==66 and data[1]==77 ):
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
		height_pixels = little_endian(data, 22, 4)
		color_planes = little_endian(data, 26, 2) # Has to be = 1
		bits_per_pixel = little_endian(data, 28, 2) # For now, only going to use 24 (8 bits per channel)
		compression_method = little_endian(data, 30, 4) # Only want to work with value = 0 (BI_RGB uncompressed data)
		image_size = little_endian(data, 34, 4) # Size of the raw bitmap data
		resolution_horizontal = little_endian(data, 38, 4) # pixels per meter
		resolution_vertical = little_endian(data, 42, 4) # pixels per meter
		num_color_palette = little_endian(data, 46, 4) # Number of colors in the color palette, from 0 to 2^n

		# Verify extracted header
		print("Bitmap Size : %d" % bitmap_size)
		print("Data start offset : %d" % start_offset)
		print("Size of the main header : %d" % header_size)
		print("Bitmap width in pixels : %d" % width_pixels)
		print("Bitmap height in pixels : %d" % height_pixels)
		print("Number of color planes : %d" % color_planes)
		print("Color depth (bits per pixel) : %d" % bits_per_pixel)
		print("Compression method : %d" % compression_method)
		print("Image Size : %d" % image_size)
		print("Horizontal resolution : %d" % resolution_horizontal)
		print("Vertical resolution : %d" % resolution_vertical)
		print("Number of colors in the color palette : %d" % num_color_palette)

		#TODO : extract bitmap data and insert into map structure

	def drawStatesMap(self, states_list, draw_params=None):
		""" Draw only a specified list of states in order, to mark a specific path """
		pass #TODO : complete this function

	def drawMap(self, draw_params=None):
		""" Draw only the static map, and nothing else """
		pass #TODO : complete this function
