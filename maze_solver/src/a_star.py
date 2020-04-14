import numpy as np
from math import sqrt


#TO DO
#Rosify with path markers
#Waypoints
#Dilate coordinates (to avoid too many vertices)
#Convexity thing to get coordinates

 ################################## BEGINNING OF QUEUE DEFINITIONS #########################################


class Queue:
	def __init__(self):
		self.items = []

	def isEmpty(self):
		return self.items == []

	def enqueue(self, item):
	    self.items.insert(0, item)

	def dequeue(self):
		return self.items.pop()

	def size(self):
		return len(self.items)


class Item:
    def __init__(self, label, key):
		self.label = label
		self.key = key


class PriorityQueue:                      # Binary Heap Implementation
    def __init__(self):                   # stores keys with unique labels
        self.A = []
        self.label2idx = {}

    def min_heapify_up(self, c):
        if c == 0: return
        p = (c - 1) // 2
        if self.A[p].key > self.A[c].key:
            self.A[c], self.A[p] = self.A[p], self.A[c]
            self.label2idx[self.A[c].label] = c
            self.label2idx[self.A[p].label] = p
            self.min_heapify_up(p)

    def min_heapify_down(self, p):
        if p >= len(self.A): return
        l = 2 * p + 1
        r = 2 * p + 2
        if l >= len(self.A): l = p
        if r >= len(self.A): r = p
        c = l if self.A[r].key > self.A[l].key else r
        if self.A[p].key > self.A[c].key:
            self.A[c], self.A[p] = self.A[p], self.A[c]
            self.label2idx[self.A[c].label] = c
            self.label2idx[self.A[p].label] = p
            self.min_heapify_down(c)

    def insert(self, label, key):         # insert labeled key
		if label in self.label2idx:
			idx = self.label2idx[label]
			old_val = self.A[idx]
			self.A[idx] = Item(label,key)
			if old_val > key:
				self.min_heapify_up(idx)
			else:
				self.min_heapify_down(idx)
		else:
			self.A.append(Item(label, key))
			idx = len(self.A) - 1
			self.label2idx[self.A[idx].label] = idx
			self.min_heapify_up(idx)

    def find_min(self):                   # return minimum key
        return self.A[0].key

    def extract_min(self):                # remove a label with minimum key
        self.A[0], self.A[-1] = self.A[-1], self.A[0]
        self.label2idx[self.A[0].label] = 0
        del self.label2idx[self.A[-1].label]
        min_label = self.A.pop().label
        self.min_heapify_down(0)
        return min_label


    def decrease_key(self, label, key):   # decrease key of a given label
        if label in self.label2idx:
            idx = self.label2idx[label]
            if key < self.A[idx].key:
                self.A[idx].key = key
                self.min_heapify_up(idx)

    def isEmpty(self):
        return len(self.A) == 0



 ################################## END OF QUEUE DEFINITIONS #########################################       


class Search:
	def __init__(self, graph, start, stop):
		'''
		graph: numpy 2D array (right now it has 1's in occupied places, and 0 in free)
		start: tuple representing start coordinates
		stop: tuple representing goal coordinates
		'''
		self.graph = graph
		self.start = start
		self.stop = stop
		print(type(self.graph))
		self.functions = {'BFS': self.BFS, 'A_Star': self.A_Star} #2 search algorithms 

	def __is_valid__(self, coord):
		'''
		Returns whether a space is occupied, NEED TO UPDATE FOR STATA BASEMENT MAP
		'''
		good = 0
		bad  = 1 #change this

		if 0 <= coord[0] < self.graph.shape[0] and 0 <= coord[1] < self.graph.shape[1]:  
			return self.graph[coord[0]][coord[1]] == good
		else:
			return False

	def __get_valid_neighbors__(self, coord):
		'''
		Gets all coordinates adjascent to the current one that are not occupied (8 at most)
		'''
		neighbors = []
		for i in range(coord[0]-1, coord[0] + 2):
			for j in range(coord[1] - 1, coord[1] + 2):
				if self.__is_valid__((i, j)):
					neighbors.append((i, j))
		return neighbors

	def __heuristic_cost__(self, a, b):
		#Manhattan Distance
		return abs(a[0] - b[0])+ abs(a[1] - b[1])


	def __heuristic_cost2__(self, a , b):
		#Euclidian Distance
		return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

	def BFS(self):
		frontier = Queue() #coordinates we need to visit
		frontier.enqueue(self.start)
		nodes_search = 1 #keeps track of our frontier size
		came_from = {}
		came_from[self.start] = None #keeps track of path 
		while not frontier.isEmpty():
			current = frontier.dequeue() #pick an element to visit
			if current == self.stop: #early finish if goal reached
				break
			for neighbor in self.__get_valid_neighbors__(current): #visit every neighbor
				if neighbor not in came_from: #if neighbor hasn't been visited yet
					nodes_search += 1
					frontier.enqueue(neighbor) #add neighbor to frontier
					came_from[neighbor] = current #add neighbor to visited nodes set
		return came_from, nodes_search


	def A_Star(self):
		'''
		Similair to BFS except we strategically chose which nodes to visit first in the frontier
		'''
		frontier = PriorityQueue()
		frontier.insert(self.start, 0)
		came_from = {}
		cost_so_far = {}
		came_from[self.start] = None
		cost_so_far[self.start] = 0
		nodes_search = 1
		while not frontier.isEmpty():
			current = frontier.extract_min() 
			if current == self.stop: 
				break
			for neighbor in self.__get_valid_neighbors__(current):
				new_cost = cost_so_far[current] + self.__heuristic_cost__(current, neighbor)
				if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
					nodes_search += 1
					cost_so_far[neighbor] = new_cost
					priority = new_cost + self.__heuristic_cost2__(self.stop, neighbor)
					frontier.insert(neighbor, priority)
					came_from[neighbor] = current

		return came_from, nodes_search



	def get_Path(self, algo):
		'''
		algo is either "BFS" or "A_Star"
		'''
		if algo not in self.functions:
			return "NOT Valid Algorithm", None, None, None


		algo = self.functions[algo]
		came_from, nodes_search = algo()
		pathgraph = self.graph.copy() #will represent path of robot using 2's #EDIT THIS TO BE WITH MARKERS
		pathgraph[self.start[0]][self.start[1]] = 2
		pathgraph[self.stop[0]][self.stop[1]] = 2
		current = self.stop
		path = []
		cost = 0
		try: 
			while current != self.start:
				path.append(current)
				pathgraph[current[0], current[1]] = 2 #CHAGNE TO MARKERS
				current = came_from[current]
				cost += self.__heuristic_cost2__(path[-1], current) 
			path.append(self.start)
			path.reverse()
			return path, nodes_search, cost, pathgraph
		except:
			return "No Path Found", nodes_search, None, None




########## Testing (without ROS) ############

def graph_generator(dimensions, obstacles):
	graph = np.zeros(dimensions)
	for coord in obstacles:
		graph[coord[0]][coord[1]] = 1
	return graph

def create_obstacles(barriers):
	obstacles = set()
	for barrier in barriers:
		for i in range(barrier[1][0], barrier[1][1]+1):
			if barrier[2] == "v": #vertical
				obstacles.add((i, barrier[0]))
			else:
				obstacles.add((barrier[0], i))
	return list(obstacles)
		

# bar1 = [2, (2, 12), 'h']
# bar2 = [12, (2, 12), 'v']
# bar3 = [12, (2, 12), 'h']
# obs = create_obstacles([bar1, bar2, bar3])

# graph = graph_generator((15, 15), obs)
# start = (12, 0)
# stop = (2, 14)

# print(type(graph))
# SearchGraph = Search(graph, start, stop)

# print('BFS')
# path_B, NS_B, DT_B, pathgraph_B = SearchGraph.get_Path("BFS")
# print('Nodes Visited:', NS_B)
# print('Distance Traveled', DT_B)
# print('Path Traveled:')
# print(pathgraph_B)

# print(" ")
# print(" __________________________________")
# print(" ")

# print('A star')
# Path_A, NS_A, DT_A, pathgraph_A = SearchGraph.get_Path("A_Star")
# print('Nodes Visited:', NS_A)
# print('Distance Traveled', DT_A)
# print('Path Traveled:')
# print(pathgraph_A)
