import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import math
import csv


# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, is_dy_obs):
        self.row = row  # coordinate
        self.col = col  # coordinate
        self.is_obs = is_obs  # obstacle?
        self.is_dy_obs = is_dy_obs  # dynamic obstacle?
        self.tag = "NEW"  # tag ("NEW", "OPEN", "CLOSED")
        self.h = 100000000000 # cost to goal (NOT heuristic)
        self.k = 100000000000    # best h
        self.parent = None  # parent node


class DStar:
    def __init__(self, grid, dynamic_grid, start, goal, is_dynamic):
        # Maps
        self.grid = grid  # the pre-known grid map
        self.dynamic_grid = dynamic_grid  # the actual grid map (with dynamic obstacles)
        # Create a new grid to store nodes
        size_row = len(grid)
        size_col = len(grid[0])
        self.grid_node = [[None for i in range(size_col)] for j in range(size_row)]
        for row in range(size_row):
            for col in range(size_col):
                self.grid_node[row][col] = self.instantiate_node((row, col))

        # The start node
        self.start = self.grid_node[start[0]][start[1]]
        # The goal node
        self.goal = self.grid_node[goal[0]][goal[1]]

        # List
        self.open = set()

        # Result
        self.path = []

        self.is_dynamic = is_dynamic

    def instantiate_node(self, point):
        ''' Instatiate a node given point (x, y) '''
        row, col = point
        node = Node(row, col, not self.grid[row][col],
                    not self.dynamic_grid[row][col])
        return node

    def get_k_min(self):
        '''Get the minimal k value from open list
        
        return:
        k - the minimal k value in the open list; 
            return -1 if the open list is empty
        '''
        # Find the node with minimal k value
        node = self.min_node()
        # If the node is None / open list is empty
        if node == None:
            return -1
        # Get the minimal k value
        else:
            return node.k

    def min_node(self):
        '''Get the node with minimal k value from open list
        
        return:
        node - the node with minimal k value in the open list; 
               return None if the open list is empty
        '''
        # If open is empty
        if len(self.open) == 0:
            return None
        # Find the minimum value
        else:
            return min(self.open, key=lambda n: n.k)

    def delete(self, node):
        ''' Remove a node from open list 
            and set it to "CLOSED"
        '''
        self.open.remove(node)
        node.tag = "CLOSED"

    def get_neighbors(self, node):
        ''' Get neighbors of a node with 8 connectivity '''
        row = node.row
        col = node.col
        neighbors = []
        # All the 8 neighbors -> (-1, 2)

        # look at more neighbors
        for i in range(-1, 2):
            for j in range(-1, 2):
                # Check range
                if row + i < 0 or row + i >= len(self.grid) or \
                        col + j < 0 or col + j >= len(self.grid[0]):
                    continue
                # Do not append the same node
                if i == 0 and j == 0:
                    continue

                neighbors.append(self.grid_node[row + i][col + j])

        return neighbors
    
    def get_sensor_neighbors(self, node):
        ''' Get neighbors of a node with 8 connectivity '''
        row = node.row
        col = node.col
        neighbors = []
        # All the 8 neighbors -> (-1, 2)

        # look at more neighbors
        for i in range(-2, 3):
            for j in range(-2, 3):
                # Check range
                if row + i < 0 or row + i >= len(self.grid) or \
                        col + j < 0 or col + j >= len(self.grid[0]):
                    continue
                # Do not append the same node
                if i == 0 and j == 0:
                    continue

                neighbors.append(self.grid_node[row + i][col + j])

        return neighbors


    def cost(self, node1, node2):
        ''' Euclidean distance from one node to another 

            return:
            distance - Euclidean distance from node 1 to node 2
                       math.inf if one of the node is obstacle
        '''
        # If any of the node is an obstacle
        if node1.is_obs or node2.is_obs:
            return 100000000000
        # Euclidean distance
        a = node1.row - node2.row
        b = node1.col - node2.col
        return (a ** 2 + b ** 2) ** (1 / 2)

    def process_state(self):
        ''' Pop the node in the open list 
            Process the node based on its state (RAISE or LOWER)
            If RAISE
                Try to decrease the h value by finding better parent from neighbors
                Propagate the cost to the neighbors
            If LOWER
                Attach the neighbor as the node's child if this gives a better cost
                Or update this neighbor's cost if it already is
        '''
        # Pop node from open list
        node = self.open.pop()
        node.tag = "CLOSED"
        kold = node.k

        # Get neighbors of the node
        # using self.get_neighbors
        neighbors = self.get_neighbors(node)

        # I HEAVILY UTILIZED THE ATTACHED SOURCE ON D*
        # In Proceedings IEEE International Conference on Robotics and Automation, May 1994.
        # Optimal and Efficient Path Planning for Partially-Known Environments
        # by Anthony Stentz
        # I attempted several times to make this function without it and kept running into annoying issues
        # So I used the pseudocode on page 3 under section 2.2 algorithm description

        if kold < node.h:
            # RAISE
            # check surrounding cells for better cost
            for near in neighbors:
                if near.h <= kold and node.h > near.h + self.cost(near, node):
                    node.h = near.h + self.cost(near, node)
                    node.parent = near

        if kold == node.h:
            # LOWER
            for near in neighbors:
                if (near.tag == "NEW") or \
                        (near.parent == node and near.h != node.h + self.cost(near, node)) or \
                        (near.parent is not node and (near.h > node.h + self.cost(near, node))):
                    self.insert(near, node.k + self.cost(node, near))
                    near.parent = node
        else:
            # Still RAISE
            for near in neighbors:
                if near.tag == "NEW" or \
                        (near.parent is node and near.h != node.h + self.cost(node, near)):
                    near.parent = node
                    self.insert(near, node.h + self.cost(near, node))
                else:
                    if near.parent is not node and near.h > node.h + self.cost(near, node):
                        self.insert(node, node.h)
                    else:
                        if near.parent is not node and node.h > near.h + self.cost(node, near) and \
                                near.tag == "CLOSED" and near.h > kold:
                            self.insert(near, near.h)
        return self.get_k_min()

    def repair_replan(self, node):
        ''' Replan the trajectory until 
            no better path is possible or the open list is empty 
        '''

        k_min = 0
        h_y = node.h
        while self.open and k_min < h_y and k_min is not 100000000000:
            k_min = self.process_state()

        if self.open:
            print("stopped looking bc k_min is less than node k")
        # Call self.process_state() until it returns k_min >= h(Y) or open list is empty
        # The cost change will be propagated

    def modify_cost(self, obstacle_node, neighbor):
        ''' Modify the cost from the affected node to the obstacle node and 
            put it back to the open list
        '''

        # Change the cost from the dynamic obstacle node to the affected node
        # by setting the obstacle_node.is_obs to True (see self.cost())

        # Im going to check if the goal is surrounded
        # kept getting an infinite loop in the path repair
        goal_neighbors = self.get_neighbors(self.goal)
        surrounded = True
        for block in goal_neighbors:
            surrounded = surrounded and block.is_obs

        # if the goal is surrounded the minimum k should be infinity because they all go through an obstacle
        if surrounded:
            print("Goal is blocked off, no path possible")
            return 100000000000

        # Put the obstacle node and the neighbor node back to Open list
        obstacle_h = self.cost(obstacle_node, obstacle_node.parent) + obstacle_node.parent.k
        self.insert(obstacle_node, obstacle_h)

        self.insert(neighbor, obstacle_node.k + self.cost(obstacle_node, neighbor))
        # for neighbors in self.get_neighbors(obstacle_node):
        #    self.insert(neighbors, neighbors.h)

        return self.get_k_min()

    def prepare_repair(self, node):
        ''' Sense the neighbors of the given node
            If any of the neighbor node is a dynamic obstacle
            the cost from the adjacent node to the dynamic obstacle node should be modified
        '''
        # Sense the neighbors to see if they are new obstacles
        neighbors = self.get_sensor_neighbors(node)

        for near in neighbors:
            near.is_dy_obs = not self.dynamic_grid[near.row][near.col]
            # If neighbor.is_dy_obs == True but neighbor.is_obs == Flase,
            
            # neighbor is a moving obstacle that is not there anymore
            if near.is_obs and not near.is_dy_obs:
                near.is_obs = False
                self.modify_cost(near, node)

            # the neighbor is a new dynamic obstacle
            if near.is_dy_obs and not near.is_obs:
                print("neighbor is new dynamic obstacle")
                near.is_obs = True
                # Modify the cost from this neighbor node to all this neighbor's neighbors
                # using self.modify_cost
                self.modify_cost(near, node)

    def insert(self, node, new_h):
        ''' Insert node in the open list

        arguments:
        node - Node to be added
        new_h - The new path cost to the goal

        Update the k value of the node based on its tag
        Append the node to the open_list
        '''
        # Update k
        if node.tag == "NEW":
            node.k = new_h
        elif node.tag == "OPEN":
            node.k = min(node.k, new_h)
        elif node.tag == "CLOSED":
            node.k = min(node.h, new_h)
        # Update h
        node.h = new_h
        # Update tag and put the node in the open set
        node.tag = "OPEN"
        self.open.add(node)

    def run(self):
        ''' Run D* algorithm
            Perform the first search from goal to start given the pre-known grid
            Check from start to goal to see if any change happens in the grid, 
            modify the cost and replan in the new map
        '''

        # Search from goal to start with the pre-known map
        self.insert(self.goal, 0)
        current_state = self.start
        # Process until open set is empty or start is reached
        while self.open:
            # using self.process_state()
            self.process_state()

        # Visualize the first path if found
        self.get_backpointer_list(self.start)
        self.draw_path(self.grid, "Path in static map")
        if self.path == []:
            print("No path is found")
            return
        
        step = 0
        if self.is_dynamic:
            self.dynamic_grid = self.get_dyn_map(step)

        for setpoint in self.path:
            if not setpoint.is_dy_obs:
                self.repair_replan(setpoint)
        # Start from start to goal
        # Update the path if there is any change in the map
        while current_state is not self.goal:
            # update its sensor values
            step += 1
            self.dynamic_grid = self.get_dyn_map(step)

            # Check if any repair needs to be done
            # using self.prepare_repair
            self.prepare_repair(current_state)

            # Replan a path from the current node
            # using self.repair_replan
            self.repair_replan(current_state)

            # Get the new path from the current node
            self.get_backpointer_list(current_state)
            # print("parent of (", current_state.col, ",", current_state.row, ") is (", current_state.parent.col, ",",
            #      current_state.parent.row, ")")

            # Uncomment this part when you have finished the previous part
            # for visualizing each move and replanning
            self.draw_path(self.dynamic_grid, "Path in progress")
            if self.path == []:
                print("No path is found")
                return

            # Get the next node to continue
            current_state = current_state.parent
            
    def get_backpointer_list(self, node):
        ''' Keep tracing back to get the path from a given node to goal '''
        # Assume there is a path from start to goal
        cur_node = node
        self.path = [cur_node]
        while cur_node != self.goal and \
                cur_node != None and \
                not cur_node.is_obs:
            # trace back
            cur_node = cur_node.parent
            # add to path
            self.path.append(cur_node)

        # If there is not such a path
        if cur_node != self.goal:
            self.path = []

################################################## dynamic map things #############################
    def get_dyn_map(self, index):
        # hand made these dynamic maps so just fix it for the application 
        index = index%35
        file_path = 'src/PathPlanning/path_planning/src/2d_imp/maps/dynamic_maps/hospital10_nocspace_dynamic'+ str(index) + '.csv'
        grid = []
        start = [0, 0]
        goal = [0, 0]
        # Load from the file
        with open(file_path, 'r') as map_file:
            reader = csv.reader(map_file)
            for i, row in enumerate(reader):
                # load start and goal point
                if i == 0:
                    start[0] = int(row[1])
                    start[1] = int(row[2])
                elif i == 1:
                    goal[0] = int(row[1])
                    goal[1] = int(row[2])
                # load the map
                else:
                    int_row = [int(col) for col in row]
                    grid.append(int_row)
        return grid

    def draw_path(self, grid, title="Path"):
        '''Visualization of the found path using matplotlib'''
        fig, ax = plt.subplots(1)
        ax.margins()

        # Draw map
        row = len(grid)  # map size
        col = len(grid[0])  # map size
        for i in range(row):
            for j in range(col):
                if not self.grid_node[i][j].is_obs:
                    ax.add_patch(Rectangle((j - 0.5, i - 0.5), 1, 1, edgecolor='w', facecolor='w'))  # free space
                else:
                    ax.add_patch(Rectangle((j - 0.5, i - 0.5), 1, 1, edgecolor='k', facecolor='k'))  # obstacle

        # Draw path
        for node in self.path:
            row, col = node.row, node.col
            ax.add_patch(Rectangle((col - 0.5, row - 0.5), 1, 1, edgecolor='b', facecolor='b'))  # path
        if len(self.path) != 0:
            start, end = self.path[0], self.path[-1]
        else:
            start, end = self.start, self.goal
        ax.add_patch(Rectangle((start.col - 0.5, start.row - 0.5), 1, 1, edgecolor='g', facecolor='g'))  # start
        ax.add_patch(Rectangle((end.col - 0.5, end.row - 0.5), 1, 1, edgecolor='r', facecolor='r'))  # goal
        # Graph settings
        plt.title(title)
        plt.axis('scaled')
        plt.gca().invert_yaxis()
        plt.show()
