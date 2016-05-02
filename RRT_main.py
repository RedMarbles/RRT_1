
from RRT_defines import *

if __name__ == "__main__":
	"""Main function"""

	#Initial and goal states
	X0 = State(0,0,0)
	Xf = State(10,10,np.pi)
	system = Plant()
	planner = Planner(initial_state=X0,final_state=Xf)

	#Create the empty tree
	T1 = Tree(X0)

	
	# Start loop
	for i in range(200)

		# Sample random point
		X_random = Plant.selectRandomPoint()

		# Find closest node 
		X_old = (planner.findClosestNode(T1, X_random))

		# Extend node in direction of point
		X_new = planner.extend(T1, X_new)  #Has to do the RRT-connect within itself
