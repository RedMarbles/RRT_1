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

