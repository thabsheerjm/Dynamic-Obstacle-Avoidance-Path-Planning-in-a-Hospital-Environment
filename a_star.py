# Basic searching algorithms REQURES A GRID MAP

# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, h):
        self.row = row          # coordinate
        self.col = col          # coordinate
        self.is_obs = is_obs    # obstacle?
        self.g = None           # cost to come (previous g + moving cost)
        self.h = h              # heuristic
        self.cost = None        # total cost (depend on the algorithm)
        self.parent = None      # previous node
        self.searched = False   # has the node been searched yet


def getneighbors(node, grid, goal, searchedNodes):
    '''Return the coordinates of neighbors of 4 of input coordinate

    arguments:
    node - Node made from a coord in the map
    grid - a nested array grid of 0's and 1's
    goal - a coordinate of the goal [4, 2] in format [row, col]

    return:
    neighbors - A nested list of Nodes that surround the input Node,
                e.g. [[0, 1], [1, 0]], returns in the order right, down, left, up'''

    coord = [node.row, node.col]
    length = len(grid)
    width = len(grid[0])

    right = [0, 1]
    down = [1, 0]
    left = [0, -1]
    up = [-1, 0]
    coordsum = [right, down, left, up]
    neighbors = []

    for direction in coordsum:
        nextto = [coord[0] + direction[0], coord[1]+direction[1]]
        # only add the coordinate to the list if it is in the grid
        if iswithingrid(nextto, length, width):
            neighbors.append(makeNode(nextto, grid, node, goal, searchedNodes))
    return neighbors


def iswithingrid(coord, length, width):
    '''Return a boolean that represents whether a coordinate is within the grid

    arguments:
    coord - node to check if it is in the map e.g. [0, 0]
    length - the number of rows in the grid (first coordinate)
    width - the number of columns in the grid (second coordinate)

    return:
     - a bool that is true if it is within the range of the grid'''

    return 0 <= coord[0] < length and 0 <= coord[1] < width


def makeNode(coord, grid, parent, goal, searchedNodes):
    '''Returns a Node class object

        arguments:
        coord - coordinates of the node
        grid - the grip map (to check if it is an obstacle)
        parent - the parent node/ previous node

        return:
         newnode -  Node class object'''

    if inNodeList(searchedNodes, coord):
        original = getnodefromlist(searchedNodes, coord)
        # the start node has no parent :'(
        if original.parent is None:
            # dumb work around but it works
            originalparentcost = 0
        else:
            originalparentcost = original.parent.cost

        # both algorithms need ot update the path and the parents if there is a faster way
        if parent.cost < originalparentcost:
            original.parent = parent
            original.cost = parent.cost + 1
        newnode = original
    else:
        obstacle = bool(grid[coord[0]][coord[1]])
        h = manhattandist(coord, goal)
        newnode = Node(coord[0], coord[1], obstacle, h)
        newnode.parent = parent
        newnode.cost = parent.cost + 1
    return newnode


def getnodefromlist(nodeList, coord):
    '''Returns a Node from a list based on coordinates

         arguments:
         nodeList - list of Nodes
         coord - a coordinate within the grid e.g. [0, 0]

         return:
          node - a Node '''
    for node in nodeList:
        if node.row == coord[0] and node.col == coord[1]:
            return node
    return None


def comparecost(node1, node2):
    '''Returns a Node

            arguments:
            node1 - first node to get the cost of
            node2 - first node to get the cost of

            return:
             node -  Node class object with the lower cost, defaults to node1 in a tie'''
    if node1.cost <= node2.cost:
        return node1
    else:
        return node2


def eucldist(start, end):
    '''Returns a value representing the euclidean distance between the coordinates

             arguments:
             start - the start coordinaes
             end - the second pair of coordinates to get the distance between

             return:
              dist - euclidean distance between coordinates '''
    # I originally was planning on using this for heuristics then read the README more carefully
    dist = ((start[1] - end[1])**2 + (start[0] - end[0])**2)**(1/2)
    return dist


def manhattandist(start, end):
    '''Returns a value representing the manhattan distance between the coordinates

             arguments:
             start - the start coordinaes
             end - the second pair of coordinates to get the distance between

             return:
              dist - manhatan distance between coordinates '''
    # this application has a simple cost because there is not difficult terrain or a different edge cost
    # so it simplifies the problem to simply manhattan distance and does not require recalculation
    # reread the readme - if each step is a cost of 1 that is just manhattan distance with neighbors of 4
    dist = abs(start[0] - end[0]) + abs(start[1] - end[1])
    return dist


def inNodeList(nodeList, coord):
    '''Returns a bool depending on if a given coordinate is already in a list of Nodes

         arguments:
         nodeList - list of Nodes
         coord - a coordinate within the grid e.g. [0, 0]

         return:
          bool - boolean if a node in the list has that coordinate '''
    for node in nodeList:
        if node.row == coord[0] and node.col == coord[1]:
            return True
    return False


def getpath(goalNode):
    '''Returns a bool depending on if a given coordinate is already in a list of Nodes

         arguments:
         nodeList - list of Nodes (assumes the last node searched is the goal)

         return:
          path - a nested list of coordiantes of type int, e.g. [[0,0], [0,1], [1,1]] '''
    goal = goalNode
    parent = goal.parent
    path = [[goal.row, goal.col]]

    while parent is not None:
        path.append([parent.row, parent.col])
        parent = parent.parent
    path.reverse()
    return path


def getcoords(nodelist):
    '''Returns a set of coordinates from the list of nodes

             arguments:
             nodeList - list of Nodes (assumes the last node searched is the goal)

             return:
             coordlist - a list of tuple coordinates, e.g. [(0,0), (0,1), (1,1)] '''
    coordlist = set()
    for node in nodelist:
        coordlist.add((node.row, node.col))
    return coordlist


def getnextnode(nodelist):
    '''Returns a Node with the lowest cost and distance to the goal sum

             arguments:
             nodeList - list of Nodes (assumes the last node searched is the goal)

             return:
             highestpriority - the node with the highest priority '''
    # default to the newest node
    highestpriority = nodelist[0]
    priorityindex = 0
    for i, node in enumerate(nodelist):
        if node.cost + node.h < highestpriority.cost + highestpriority.h:
            highestpriority = node
            priorityindex = i

    return highestpriority, priorityindex


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### I did look at https://www.redblobgames.com/pathfinding/a-star/introduction.html but this code already
    # diverged too much for me to use any code that was too similiar, but its a cool site anyway

    path = []
    steps = 0
    found = False
    # initialize the start
    startNode = Node(start[0], start[1], grid[start[0]][start[1]], 0)
    startNode.searched = True
    startNode.cost = 0
    searchedNodes = [startNode]
    # i called it queue in the other function but it works better in my head to be called a frontier here
    frontier = getneighbors(startNode, grid, goal, searchedNodes)

    while frontier:
        nextinqueue, priorityi = getnextnode(frontier)
        frontier.pop(priorityi)
        # if the next location has not been searched before and is not an obstacle then go into the process
        if not nextinqueue.searched and not nextinqueue.is_obs:
            # now the object will be searched
            nextinqueue.searched = True
            searchedNodes.append(nextinqueue)

            # check if it is the goal
            if nextinqueue.row == goal[0] and nextinqueue.col == goal[1]:
                # yay found the goal, break the for loop
                found = True
                break
            # otherwise look other places
            else:
                # update the frontier
                nextwave = getneighbors(nextinqueue, grid, goal, searchedNodes)
                frontier += nextwave

    if found:
        path = getpath(nextinqueue)
        # used to visualize the searched area
        # path = getcoords(searchedNodes)
        steps = len(getcoords(searchedNodes))
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, steps
