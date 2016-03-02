# RRT_1

This repository is meant to implement an RRT-based kinodynamic planner for a Vehicle type object. The aim is to make an intermediate-level planner that connects a global low-frequency GPS planner with a local obstacle-avoidance level high-frequency deterministic planner.

Current work to be done for this code:

* Implement a map class that can
	* Store a multi-dimensional map as a discrete set of points
	* Store a 2-D occupancy/obstacle grid on this map
	* Implement APIs to interface with the RRT algorithm (interface to be defined)
	* Set up a GUI to create your own obstacle map and start/goal configurations
	* Create output functions to print the map along with a path and start/goal configs

* Implement a deterministic local planner that can reach a given (x,y,theta) configuration
