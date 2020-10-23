from search import *
import sys

if __name__ == '__main__':
# defining the problem class from search.py to be used for our assignment to generate the results
    class search(Problem):
        """The abstract class for a formal problem. You should subclass
        this and implement the methods actions and result, and possibly
        __init__, goal_test, and path_cost. Then you will create instances
        of your subclass and solve them with the various search functions."""

        def __init__(self, initial, goal=None):
            """The constructor specifies the initial state, and possibly a goal
            state, if there is a unique goal. Your subclass's constructor can add
            other arguments."""
            self.initial = initial
            self.goal = goal
 
        def actions(self, state):
            """Return the actions that can be executed in the given
            state. The result would typically be a list, but if there are
            many actions, consider yielding them one at a time in an
            iterator, rather than building them all at once."""
            self.act = [i[0] for i in map_inp[state]]
            # print("act", self.act)
            return self.act

        def result(self, state, action):
            """Return the state that results from executing the given
            action in the given state. The action must be one of
            self.actions(state)."""
            return action

        def goal_test(self, state):
            """Return True if the state is a goal. The default method compares the
            state to self.goal or checks for state in self.goal if it is a
            list, as specified in the constructor. Override this method if
            checking against a single self.goal is not enough."""
            if isinstance(self.goal, list):
                return is_in(state, self.goal)
            else:
                return state == self.goal

        def path_cost(self, c, state1, action, state2):
            """Return the cost of a solution path that arrives at state2 from
            state1 via action, assuming cost c to get up to state1. If the problem
            is such that the path doesn't matter, this function will only look at
            state2. If the path does matter, it will consider c and maybe state1
            and action. The default method costs 1 for every step in the path."""
            for i in map_inp[state1]:
                if(i[0] == state2):
                    return c+i[1]

        def h(self, node):
            if(node.state in heuristic):
                return heuristic[node.state]
            else:
                return float('inf')

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
# taking input file from the cmd
    input_file = sys.argv[1]
# reading the input file line by line to save the input graph into a dictionary data structure
    input_file = open(input_file).readlines()
# taking input from the cmd to save the algorithm using which the solution needs to be generated
    search_algo_str = sys.argv[2]
# counter variable to keep track of the position in the input file
    pointer=-1
# defining the dictionary data structure to store the input graph
    map_inp = dict()
# defining the dictionary data structure to store the input heuristic function for the problem
    heuristic = dict()
# before and current pointer to traverse the input file
    before = ''
    current = ''

    for i in input_file:
        current = i[0]
#slpitting the input file on white spaces
        t = i.split(" ")
        if(before == '#' and not current == '#'):
            pointer+=1
# storing the input graph on encountering the first sentence after the initial comments
        if not i.startswith('#') and pointer== 0:
            if(t[0] not in map_inp):
                map_inp[t[0]] = []
            map_inp[t[0]].append([t[1], int(t[3][:-1])])
# checking if the edge is bidirectional or not and saving the information accordingly
            if(t[2] == "<>"):
                if(t[1] not in map_inp):
                    map_inp[t[1]] = []
                    map_inp[t[1]].append([t[0], int(t[3][:-1])])
# storing the start and end goal node on encountering the next comment
        elif not i.startswith('#') and pointer==1:
            begin = str(t[0])
            goal= str(t[1][:-1])
# storing the heuristic function on encountering the next comment
        elif not i.startswith('#') and pointer==2:
            heuristic[t[0]] = (int(t[1].strip('\n')))
        before = i[0]
   
    travel_problem = search(begin, goal)
# calling the respective traversal algorithm based on the input
# breadth_first_tree_search
    if search_algo_str == 'BFTS':
        goal_node = breadth_first_tree_search(travel_problem)
# breadth_first_graph_search
    elif search_algo_str == 'BFGS':
        goal_node = breadth_first_graph_search(travel_problem)
# depth_first_tree_search
    elif search_algo_str == 'DFTS':
        goal_node = depth_first_tree_search(travel_problem)
# depth_first_graph_search
    elif search_algo_str == 'DFGS':
        goal_node = depth_first_graph_search(travel_problem)
# greedy_best_first_tree_search
    elif search_algo_str == 'GBFTS':
        h=None
        h=memoize(h or travel_problem.h, 'h')
        goal_node = best_first_tree_search(travel_problem, lambda node: h(node))
# greedy_best_first_graph_search
    elif search_algo_str == 'GBFGS':
        h=None
        h=memoize(h or travel_problem.h, 'h')
        goal_node = greedy_best_first_graph_search(travel_problem, lambda node: h(node))
# uniform_cost_tree_search
    elif search_algo_str == 'UCTS':
        goal_node = uniform_cost_search(travel_problem, lambda node: h(node))
# uniform_cost_graph_search
    elif search_algo_str == 'UCGS':
        goal_node = uniform_cost_search(travel_problem, lambda node: node.path_cost)
# Astar_tree_search
    elif search_algo_str == 'ASTS':
        goal_node = astar_search(travel_problem)
# Astar_graph_search
    elif search_algo_str == 'ASGS':
        goal_node = astar_search(travel_problem, lambda node: node.path_cost)
# printing the goal node
    print("Goal Node", goal_node)

    # Do not change the code below.
    if goal_node is not None:
        print("Solution path", goal_node.solution())
        print("Solution cost", goal_node.path_cost)
    else:
        print("No solution was found.")