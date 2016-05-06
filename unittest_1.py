"""
Unit test of the functions
"""

from RRT_plant import *
from RRT_controller import PlannerRRTSimple_d1
import pdb


def test_calculateControl():
	X0 = State(0,0,0)
	Xf = State(10,10,np.pi/2)
	planner = PlannerRRTSimple_d1(initial_state=X0,final_state=Xf,mapfile="Maps/Map1.bmp")

	print("\n\n Begin debug")
	planner.model['dt'] = 1 # [s]
	X_new = planner.selectNextTarget()
	node_old = planner.findClosestNode(X_new)
	X_old = planner.tree[node_old].state

	X_old = State(1,1,0)
	X_new = State(9,9,np.pi/2)
	U = planner.calculate_control(X_old, X_new)
	print(U)
	X_reach =  planner.update(X_old, U)
	print(X_reach)


if __name__ == "__main__":
	pdb.run('test_calculateControl()')
	#test_calculateControl()
