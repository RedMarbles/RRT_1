
from RRT_plant import *
from RRT_controller import *

if __name__ == "__main__":
	"""Main function"""

	#Initial and goal states
	X0 = State(0,0,0)
	Xf = State(10,10,np.pi/2)
	planner = Planner(initial_state=X0,final_state=Xf)

	
	# Start loop
	for i in range(200):

		# Sample random point
		X_random = planner.selectNextTarget()

		# Grow tree in new direction
		reached_goal = planner.grow(X_random)

		if(reached_goal):
			break

	planner.map.drawMap()
	planner.map.drawTree(states_tree=planner.tree)
	planner.map.show()
