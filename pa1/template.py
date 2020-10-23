from search import best_first_tree_search, best_first_graph_search, uninformed_tree_search
from search import uniform_cost_search, greedy_best_first_search, astar_search, breadth_first_search, depth_first_search

import sys

if __name__ == '__main__':
	
	input_file = open(travel-input.txt)
	search_algo_str = sys.argv[2]
	
	# TODO implement
	
	goal_node = ... # TODO call the appropriate search function with appropriate parameters
	
	# Do not change the code below.
	if goal_node is not None:
		print("Solution path", goal_node.solution())
		print("Solution cost", goal_node.path_cost)
	else:
		print("No solution was found.")