"""
Defines the map class and functions
This implementation of the map class uses a gridworld representation
"""

import numpy as np
from matplotlib import pyplot as plt

class Map:
	"""Represents the full state and interface of the map"""

	default_params = { 	'bot_length':0.5, 
						'bot_color':'b',
						'path_color':'b' }

	def __init__(self, mapfile="Maps/Map1.bmp"):
		""" Map class Constructor """
		#TODO:
		# Choose to update default values for the following:
		# * map_limits
		self.map_grid = self.loadBitmap(mapfile)
		self.map_width = len(self.map_grid[0])
		self.map_height = len(self.map_grid)
		self.res_x = 0.1 # [m] - Distance on ground that each pixel represents
		self.res_y = 0.1 # [m] - Distance on ground that each pixel represents
		self.map_limits = {}
		self.map_limits['xmin'] = 0
		self.map_limits['xmax'] = self.map_width * self.res_x
		self.map_limits['ymin'] = 0
		self.map_limits['ymax'] = self.map_height * self.res_y
		#self.drawMap()

	def addRectObstacle(self, etc):
		""" Adds a rectangular obstacle to the map """
		pass #TODO

	def inputObstacles(self, etc):
		""" Takes input from the user to draw obstacles """
		pass #TODO

	def checkCollision(self, x, y):
		""" Checks if there is an obstacle at the specified location """
		#Convert real location to matrix location
		row = int( np.round(y/self.res_y) )
		col = int( np.round(x/self.res_x) )

		#Check bounds of requested points
		if ( (row<0) or (row>=len(self.map_grid)) ):
			return False
		if ( (col<0) or (col>=len(self.map_grid[0])) ):
			return False

		#Check value of map at point
		if (self.map_grid[row][col] == 0):
			return True
		else:
			return False


	def drawTree(self, states_tree, draw_params=None, figure_number=1):
		""" Accepts tree of states, and outputs the map with obstacles and the accepted states 
		    states_tree : Tree object 
		    draw_params : Dictionary that holds the parameters of the draw function"""
		# Merge the default parameters with the argument
		params = Map.default_params.copy()
		if(draw_params!=None):
			params.update(draw_params)

		plt.figure(figure_number)
		
		# Go backwards through the tree
		for i in range( len(states_tree)-1, 0, -1 ) :
			start = states_tree[i].state
			end = states_tree[ states_tree[i].parent ].state
			if ( np.abs(states_tree[i].control.omega) >= 4 ):
				path_color = 'blue' #magenta'
			else:
				path_color = 'blue'
			#plt.plot([start.x, end.x],[start.y, end.y],color=params['path_color'])
			plt.plot([start.x, end.x],[start.y, end.y],color=path_color)

		plt.axis([ self.map_limits['xmin'], self.map_limits['xmax'], self.map_limits['ymin'], self.map_limits['ymax'] ])
		#plt.show()
		return


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
			print("ERROR: File is not a bitmap")
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
			print("ERROR: Bitmap unknown format")
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

		# Verify file is in desired format
		if not((color_planes==1) and (bits_per_pixel==24) and (compression_method==0) and ((bitmap_size-start_offset)%height_pixels==0) ):
			print("ERROR: Bitmap unusable format")
			return 0

		# Extract data (working with local variable)
		rowstep = (bitmap_size - start_offset) // height_pixels # Number of bytes between successive rows
		numchannels = 3
		map_mat = []
		for row in range(height_pixels):
			row_list = []
			for col in range(width_pixels):
				row_list.append( data[start_offset + row*rowstep + col*numchannels]//255 )
			map_mat.append(row_list)
		#for row in range(len(map_mat)):
		#	print(map_mat[row])

		return map_mat



	def drawPath(self, states_list, draw_params=None, figure_number=1):
		""" Draw only a specified list of states in order, to mark a specific path
		    states_list : List object 
		    draw_params : Dictionary that holds the parameters of the draw function"""
		# Merge the default parameters with the argument
		params = Map.default_params.copy()
		if(draw_params!=None):
			params.update(draw_params)

		plt.figure(figure_number)
		
		# Loop through list of states
		for i in range( len(states_list)-1 ) :
			start = states_list[i]
			end = states_list[ i+1 ]
			plt.plot([start.x, end.x],[start.y, end.y],color=params['path_color'])

		plt.axis([ self.map_limits['xmin'], self.map_limits['xmax'], self.map_limits['ymin'], self.map_limits['ymax'] ])
		#plt.show()
		return

	def drawMap(self, draw_params=None, figure_number=1):
		""" Draw only the static map, and nothing else """

		plt.figure(figure_number)

		for row in range(self.map_height):
			print("Plotting line %d of %d" % (row, self.map_height))
			for col in range(self.map_width):
				pix = self.map_grid[row][col]
				color = 'white' if (pix==1) else 'black'
				if(color=='white') :
					continue
				x_list = [ self.res_x*col, self.res_x*(col+1), self.res_x*(col+1), self.res_x*col ]
				y_list = [ self.res_y*row, self.res_y*row, self.res_y*(row+1), self.res_y*(row+1) ]
				plt.fill(x_list, y_list, color=color)
		
		plt.axis([ self.map_limits['xmin'], self.map_limits['xmax'], self.map_limits['ymin'], self.map_limits['ymax'] ])
		#plt.show()
		return

	def drawControls(self, controls_list, draw_params=None, figure_number=2):
		""" Draws the plot of the controls """
		# Merge the default parameters with the argument
		params = Map.default_params.copy()
		if(draw_params!=None):
			params.update(draw_params)

		plt.figure(figure_number)

		v_list = []
		w_list = []
		l = len(controls_list)
		for i in range(l):
			U = controls_list[l-i-1]
			v_list.append(U.v)
			w_list.append(U.omega)

		plt.subplot(2,1,1)
		plt.plot(v_list)
		plt.ylabel('Velocity (m/s)')
		plt.axis([ 0, len(v_list), np.min(v_list)-1, np.max(v_list)+1])
		plt.subplot(2,1,2)
		plt.plot(w_list)
		plt.ylabel('Rotational velocity (rad/s)')
		plt.xlabel('Time instant')
		plt.axis([ 0, len(w_list), np.min(w_list)-1, np.max(w_list)+1])

		# Loop through list of states
		# for i in range( len(controls_list)-1 ) :
		# 	start = controls_list[i]
		# 	end = controls_list[ i+1 ]
		# 	plt.subplot(2,1,1)
		# 	plt.plot([i, i+1],[start.v, end.v],color=params['path_color'])
		# 	plt.subplot(2,1,2)
		# 	plt.plot([i, i+1],[start.omega, end.omega],color=params['path_color'])

		#plt.axis([ self.map_limits['xmin'], self.map_limits['xmax'], self.map_limits['ymin'], self.map_limits['ymax'] ])
		#plt.show()
		return

	def show(self):
		plt.show()

