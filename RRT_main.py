
from RRT_plant import *
from RRT_controller import Planner
from RRT_controller import PlannerRRTSimple_d1

if __name__ == "__main__":
	"""Main function"""

	#Initial and goal states
	#X0 = State(0,0,0)
	#Xf = State(9,9,np.pi/2)
	X0 = State(1,1,0)
	Xf = State(3,28,np.pi/2)
	#planner = Planner(initial_state=X0,final_state=Xf,mapfile="Maps/Map3.bmp")
	planner = PlannerRRTSimple_d1(initial_state=X0,final_state=Xf,mapfile="Maps/Map3.bmp")

	
	tree_size_count = 100
	# Run loop until we reach the goal or the tree has a maximum size
	while( len(planner.tree)<1000 ):

		# Sample random point
		X_random = planner.selectNextTarget()

		# Grow tree in new direction
		reached_goal = planner.grow(X_random)

		if( len(planner.tree)>tree_size_count ):
			print("%d nodes" % tree_size_count)
			tree_size_count += 100

		if(reached_goal):
			break

	print("Calculated tree of size %d nodes" % len(planner.tree) )
	print("Plotting map ...")
	planner.map.drawMap()
	planner.map.drawTree(states_tree=planner.tree)
	if (reached_goal):
		print("Successfully reached goal with tree of size %d" % len(planner.tree) )
		planner.map.drawPath( planner.tree.getPathOfStates( len(planner.tree)-1 ), {'path_color':'red'})
		planner.map.drawControls( planner.tree.getPathOfControls( len(planner.tree)-1 ) )
	print("All processes complete. Close the figure to finish the program.")
	planner.map.show()

	g = [node.control.omega for node in planner.tree ]
	print("g max : %f " % max(g))
	print("g min : %f " % min(g))
