
from RRT_defines import *

if __name__ == "__main__":
	"""Main function"""

	#Initial and goal states
	X0 = State(0,0,0)
	Xf = State(10,10,np.pi)
	system = Plant()

	#Create the empty tree
	T1 = Tree(X0)

	//