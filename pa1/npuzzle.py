from search import *
 # TODO import the necessary classes and methods
import sys

if __name__ == '__main__':

class npuzzle(Problem):
    """ The problem of sliding tiles numbered from 1 to 8 on a 3x3 board, where one of the
    squares is a blank. A state is represented as a tuple of length 9, where  element at
    index i represents the tile number  at index i (0 if it's an empty square) """

    def __init__(self, initial, goal= puzzle):
        """ Define goal state and initialize a problem """
        super().__init__(initial, goal)

    def find_blank_square(self, state):
        """Return the index of the blank square in a given state"""

        return state.index(0)

    def actions(self, state):
        """ Return the actions that can be executed in the given state.
        The result would be a list, since there are only four possible actions
        in any given state of the environment """
        n=puzzle.length()
        possible_actions = ['UP', 'DOWN', 'LEFT', 'RIGHT']
        index_blank_square = self.find_blank_square(state)
        if index_blank_square % n == 0:
            possible_actions.remove('LEFT')
        if index_blank_square < n:
            possible_actions.remove('UP')
        if index_blank_square % n == 2:
            possible_actions.remove('RIGHT')
        if index_blank_square > n:
            possible_actions.remove('DOWN')

        return possible_actions

    def result(self, state, action):
        """ Given state and action, return a new state that is the result of the action.
        Action is assumed to be a valid action in the state """

        # blank is the index of the blank square
        blank = self.find_blank_square(state)
        new_state = list(state)

        delta = {'UP': -3, 'DOWN': 3, 'LEFT': -1, 'RIGHT': 1}
        neighbor = blank + delta[action]
        new_state[blank], new_state[neighbor] = new_state[neighbor], new_state[blank]

        return tuple(new_state)

    def goal_test(self, state):
        """ Given a state, return True if state is a goal state or False, otherwise """

        return state == self.goal

    def check_solvability(self, state):
        """ Checks if the given state is solvable """

        inversion = 0
        for i in range(len(state)):
            for j in range(i + 1, len(state)):
                if (state[i] > state[j]) and state[i] != 0 and state[j] != 0:
                    inversion += 1

        return inversion % 2 == 0

    def h(self, node):
        """ Return the heuristic value for a given state. Default heuristic function used is
        h(n) = number of misplaced tiles """

        return sum(s != g for (s, g) in zip(node.state, self.goal))



# defining best_first_tree_search function to traverse the graph using the best first tree search algorithm
    

    def best_first_tree_search(problem, f, display = False):
        print("before", f)
        f=memoize(f,'f')
        print("f",f)
        node = Node(problem.initial)
        frontier = PriorityQueue('min', f)
        frontier.append(node)
        while frontier:
            node = frontier.pop()
            if problem.goal_test(node.state):
                return node
            for child in node.expand(problem):
                if child not in frontier:
                    frontier.append(child)
                if child in frontier:
                    if f(child) < frontier[child]:
                        del frontier[child]
                        frontier.append(child)

        return None

    travel_problem = npuzzle(begin, goal)

	
	input_file = sys.argv[1]
    input_file = open(input_file).readlines()
    search_algo_str = sys.argv[2]
    puzzle = []
    textFile = open("npuzzle-input.txt")
	lines = textFile.readlines()
	for line in lines:
    	if not i.startswith('#'):
    		puzzle = [line.split() for line in input_file]
           	
#TODO implement

    if search_algo_str == 'BFTS':
        goal_node = breadth_first_tree_search(travel_problem)
    elif search_algo_str == 'BFGS':
        goal_node = breadth_first_graph_search(travel_problem)
    elif search_algo_str == 'DFTS':
        goal_node = depth_first_tree_search(travel_problem)
    elif search_algo_str == 'DFGS':
        goal_node = depth_first_graph_search(travel_problem)
    elif search_algo_str == 'GBFTS':
        h=None
        h=memoize(h or travel_problem.h, 'h')
        goal_node = best_first_tree_search(travel_problem, lambda node: h(node))
    elif search_algo_str == 'GBFGS':
        h=None
        h=memoize(h or travel_problem.h, 'h')
        goal_node = greedy_best_first_graph_search(travel_problem, lambda node: h(node))
    elif search_algo_str == 'UCTS':
        goal_node = uniform_cost_search(travel_problem, lambda node: h(node))
    elif search_algo_str == 'UCGS':
        goal_node = uniform_cost_search(travel_problem, lambda node: node.path_cost)
    elif search_algo_str == 'ASTS':
        goal_node = astar_search(travel_problem)
    elif search_algo_str == 'ASGS':
        goal_node = astar_search(travel_problem, lambda node: node.path_cost)

    print("Goal Node", goal_node)

	# Do not change the code below.
	if goal_node is not None:
		print("Solution path", goal_node.solution())
		print("Solution cost", goal_node.path_cost)
	else:
		print("No solution was found.")
