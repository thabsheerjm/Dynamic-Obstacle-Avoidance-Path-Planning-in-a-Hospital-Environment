import rospy
from nav_msgs import OccupancyGrid, GetMap
from d_star import DStar
import math

# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, is_dy_obs):
        self.row = row  # coordinate
        self.col = col  # coordinate
        self.is_obs = is_obs  # obstacle?
        self.is_dy_obs = is_dy_obs  # dynamic obstacle?
        self.tag = "NEW"  # tag ("NEW", "OPEN", "CLOSED")
        self.h = math.inf  # cost to goal (NOT heuristic)
        self.k = math.inf  # best h
        self.parent = None  # parent node

class DStar:
    def __init__(self, grid, dynamic_grid, start, goal):
        # Maps
        rospy.wait_for_service('add_two_ints')
        self.grid = grid
        # the pre-known grid map
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

def setup():
    rospy.init_node('dStar', anonymous=True)
    rospy.Subscriber("getMap", OccupancyGrid, load_map) 
    rospy.wait_for_service('/static_map')
    try:
        get_map = rospy.ServiceProxy('/static_map', GetMap)
        grid = get_map()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        grid = []

    return grid


# Load map, start and goal point.
def load_map(data):
    # TODO: Change this into a service call and adjust start and goal
    grid = data.data


if __name__ == "__main__":
    

    # Load the map
    grid = setup()
   
    # TODO: adjust dynamic grid to be sensor values
    # Search
    d_star = DStar(grid)

    # Run D*
    # d_star.run()
    print("test")
