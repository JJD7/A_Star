import numpy as np
import math
import pygraphviz as graph
import networkx as nx
import matplotlib.pyplot as plt

class path_planner:

    def __init__(self, data_file, coord_file):
        '''#####################################################################
            For every class instance, need to parse the data
            extracting node count, start node, and goal from the
            first three lines. The rest of the lines (4->inf) is edge info.
        #####################################################################'''
        f = open(data_file)
        self.n = int(f.readline())
        self.start = int(f.readline())
        self.goal = int(f.readline())
        f.close
        self.weighted_edge_list = np.genfromtxt(data_file, delimiter=' ', skip_header=3)
        self.coords_data = np.genfromtxt(coord_file, delimiter=' ')
        self.rows = self.n
        self.columns = self.rows -1
        self.edges = {}
        self.coords = {}
        self.color_map = []
        self.O = {self.start:0.0}           #open list starts with starting node
        self.C = []                         #closed list is initially empty
        self.B = {self.start:self.start}    #backpointer dictionary. For now, start node points to itself
        self.V = {}                         #empty cost dictionary to be expanded later
        self.expansions = 0
        self.alpha = 1
        self.shortest_path = [self.goal]
        for x in range(1,self.n+1):
            if x == self.start: self.V[x] = 0.0
            else: self.V[x] = math.inf

    def create_DAG(self):
        '''##########################################
            Creates a graph object primarily used for
            visulaization. Also makes indexing edge
            cost a little more convenient
        ##########################################'''
        for x in range(1,self.n+1): #Moving the parst data from lists to dictionaries.
            #not strictly necessary, but makes lookup easier
            ix = np.isin(self.weighted_edge_list[:,0],x)
            self.edges[x] = np.where(ix)
            self.coords[x] = self.coords_data[x-1]

        self.DAG = nx.Graph()

        for node in range(1,self.n+1): #add edges to DAG with weights
            for x in range(len(self.edges[node][0])):
                xj = int(self.weighted_edge_list[self.edges[node][0][x],1])
                w = self.weighted_edge_list[self.edges[node][0][x],2]
                self.DAG.add_edge(node, xj, weight = w)

    def huristic(self, node, plan_type):
        if plan_type: h=0   #when using dikstra's Algorithm, the huristic is zero
        else: #use euclidian distance in A* Algorithm
            dx = self.coords[self.goal][0]-self.coords[node][0]
            dy = self.coords[self.goal][1]-self.coords[node][1]
            h = self.alpha*math.sqrt(dx**2+dy**2)
        return h

    def planner(self, plan_type):   #actual planning Algorithm executes here
        V_new = 0.0
        while self.goal not in self.C:
            #get the node in the open list with the minimum cost to come plus huristic
            #add this node to closed list and remove from open list.
            xj = min(self.O, key=lambda i: self.V[i] + self.huristic(i, plan_type))
            self.C.append(xj)
            del self.O[xj]
            self.expansions +=1         #incrementer to track performance

            if xj == self.goal: #this is the stop condition for the Algorithm
                print('goal reached in ', self.expansions,  'expansions')
                print('final cost is: ', self.V[self.goal])
                break

            edge_index = self.edges[xj] #prepare a list of edges connected to node


            #ignore neighboring nodes already in closed list
            #add neighbors to open list and compute the cost to come for each node
            for x in range(len(edge_index[0])):
                xi = self.weighted_edge_list[edge_index[0][x],1]
                if xi in self.C: continue
                if xi not in self.O: self.O[xi]=0.0
                V_new = self.DAG[xj][xi]['weight'] + self.V[xj]

                #record the lowest cost so far
                #set backpoint to track shortest path so far
                if V_new < self.V[xi]:
                    self.V[xi] = V_new
                    self.B[xi] = xj
                    self.O[xi] = V_new

            if self.O is []:
                print("could not find path to goal")

        #build shortest path from the backpointer dictionary working back from goal.
        pointer = self.goal

        #print('Shortest Path is:\n',goal)
        while pointer != self.start:
            #print(B[pointer])
            self.shortest_path.append(self.B[pointer])
            pointer = self.B[pointer]

    def plot_path(self, sub):
        #build a color map to indicate nodes on frontier, nodes visited
        #and nodes on the shortest path.
        for node in self.DAG:
            if node == self.start:
                self.color_map.append('green')
            elif node == self.goal:
                self.color_map.append('red')
            elif node in self.shortest_path: self.color_map.append('red')
            elif node in self.O: self.color_map.append('blue')
            elif node in self.C: self.color_map.append('grey')
            else: self.color_map.append('lightgrey')
        plt.subplot(2, 1, sub+1)
        nx.draw_networkx_nodes(self.DAG, self.coords, with_labels=False, node_color = self.color_map, node_size = 40)
        nx.draw_networkx_edges(self.DAG, self.coords, edge_color = 'lightblue')
        #nx.draw_networkx_labels(DAG, coords)



def main():
    input_files = ["input_1.txt", "input_2.txt", "input_3.txt"]
    coord_files = ["coords_1.txt", "coords_2.txt", "coords_3.txt"]
    legend = ['A* Algorithm', 'Dikstra Algorithm']
    plotting = False

    output_costs = open("output_costs.txt", 'w')
    output_numiters = open('output_numiters.txt', 'w')

    for x in range(3):
        if plotting: plt.figure()

        for z in range(1,-1,-1):
            problem = path_planner(input_files[x], coord_files[x])
            problem.create_DAG()
            problem.planner(z)
            output_costs.write(str(problem.V[problem.goal]))
            output_costs.write('\t')
            output_numiters.write(str(problem.expansions))
            output_numiters.write('\t')

            if plotting:
                problem.plot_path(z)
                ax = plt.gca()
                ax.set_ylabel(legend[z])
                ax.spines['top'].set_visible(False)
                ax.spines['left'].set_visible(False)
                ax.spines['right'].set_visible(False)
                ax.spines['bottom'].set_visible(False)
                ax.tick_params(bottom=False, left=False)
                ax.set_yticklabels([])
                ax.set_xticklabels([])

        output_costs.write('\n')
        output_numiters.write('\n')

        if plotting:ax.set_title(input_files[x])
    #plot the network
    if plotting: plt.show()

    output_costs.close
    output_numiters.close

if __name__== "__main__":
    main()
