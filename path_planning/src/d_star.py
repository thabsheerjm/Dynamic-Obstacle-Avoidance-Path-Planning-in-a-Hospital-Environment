#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist, Pose
from nav_msgs.msg import OccupancyGrid, Path, MapMetaData

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math


# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, is_dy_obs, cost):
        self.row = row  # coordinate
        self.col = col  # coordinate
        self.is_obs = is_obs  # obstacle?
        self.is_dy_obs = is_dy_obs  # dynamic obstacle?
        self.tag = "NEW"  # tag ("NEW", "OPEN", "CLOSED")
        self.h = 100000000  # cost to goal (NOT heuristic)
        self.k = 100000000  # best h
        self.parent = None  # parent node
        self.cost = cost # cost map value between 0 and 100 (% confidence it is obstacle)


class DStar:
    def __init__(self, grid= [[0, 0], [0, 0]], dynamic_grid = [[0]], start=[0, 0], goal=[0, 0]):
        #TODO: set up publishers and subscribers to these topics
        # (make the subscribers update each thing in the handlers)
        
        rospy.loginfo("initializing d*") 

        # Maps
        self.grid = grid  # the pre-known grid map
        self.gridinfo = MapMetaData()
        self.resolution_mod = 2.0

        self.dynamic_grid = dynamic_grid  # the actual grid map (with dynamic obstacles)
        self.dynamic_gridinfo = None
        self.dynamic_row = len(grid)
        self.dynamic_col = len(grid[0])


        # Create a new grid to store nodes
        size_row = len(grid)
        size_col = len(grid[0])
        
        self.grid_node = [[self.instantiate_node((i, j), 0) for i in range(size_col)] for j in range(size_row)]

        # The start node
        self.start = self.grid_node[start[0]][start[1]] # where the robot is in grid coordinates
        self.current_state = Pose() # where the robot is in global coordinates

        # The goal node
        self.goal = self.grid_node[goal[0]][goal[1]] # goal in grid coords
        self.goal_pose = [] # goal in global coords

        # List
        self.open = set()

        # Result
        self.path = []
        self.ros_path = Path()
        self.path_list = [PoseStamped()]

        # cut down on the grid and only look at a middle square
        # for 3,  min 30, max 70 for both
        # for 2, min 70 (row), 100 (col), max 155 both
        self.row_min = 270/(self.resolution_mod*self.resolution_mod) # 280
        self.row_max = 630/(self.resolution_mod*self.resolution_mod) # 615
        self.col_min = 270/(self.resolution_mod*self.resolution_mod)
        self.col_max = 630/(self.resolution_mod*self.resolution_mod)

        ############################################ ROS THINGS ###############################

        rospy.loginfo("setting up subscribers")   

        #subscribers
        # /gopher_presence/move_base/global_costmap/costmap --> grid (OccupancyGrid)
        rospy.Subscriber("/gopher_presence/move_base/global_costmap/costmap", OccupancyGrid, self.make_grid)
        # /gopher_presence/move_base/local_costmap/costmap --> dynamic_grid (OccupancyGrid)
        # rospy.Subscriber("/gopher_presence/move_base/local_costmap/costmap", OccupancyGrid, self.make_dynamicGrid)
        # /model_pose # Pose Stamped (start/current state)
        rospy.Subscriber("/model_pose", PoseStamped, self.update_start)

        rospy.Subscriber("/gopher_presence/move_base_simple/goal", PoseStamped, self.update_goal)

        rospy.loginfo("setting up publishers")
        # /gopher_presence/move_base/TrajectoryPlannerROS/global_plan (copy for visualization, Path)
        self.pathPub = rospy.Publisher('/gopher_presence/d_star/path', Path, queue_size=10)
        # /gopher_presence/base_controller/cmd_vel (twist)
        self.velPub = rospy.Publisher('/gopher_presence/base_controller/cmd_vel', Twist, queue_size= 10)
        self.movebase_pathpub = rospy.Publisher('/gopher_presence/move_base/TrajectoryPlannerROS/global_plan', Path, queue_size=10)
        

    def update_goal(self, data):
        '''subscriber function to get the goal
        '''
        rospy.logwarn("Recieved a goal at:")
        rospy.loginfo(data.pose.position)
        rospy.logwarn("in the grid coordinates it is at:")
        (row, col) = self.global2grid((data.pose.position.x, data.pose.position.y))
        grid_goal = [row, col]
        rospy.loginfo(grid_goal)

        if (row < len(self.grid_node) and col < len(self.grid_node[0])):
            self.goal = self.grid_node[grid_goal[0]][grid_goal[1]]
        else:
            rospy.logwarn("The grid is not big enough, the goal was not chosen right")
            self.goal = self.grid_node[0][0]
        self.goal_pose = data.pose
        
        # reset the path and the checked nodes
        self.path = []
        self.open = set()
        # stop moving to recalc whats happening
        self.stopRobot()
        # actually run the code now that is has a goal
        self.run()

    def stopRobot(self):
        '''stops the robot from moving'''
        rospy.logwarn("WAIT, STOP THE ROBOT")
        stop = Twist()
        stop.linear.x = 0.0
        stop.linear.y = 0.0
        stop.linear.z = 0.0
        stop.angular.x = 0.0
        stop.angular.y = 0.0
        stop.angular.z = 0.0
        self.velPub.publish(stop)

    def update_start(self, data):
        ''' subcriber function attached to /model_pose
            PoseStamped, in gobal coordinate frame 
        '''
        old_start = self.global2grid((self.current_state.position.x, self.current_state.position.y))
        grid_start = self.global2grid((data.pose.position.x, data.pose.position.y))
        # rospy.logwarn("looking for start, the grid size is: " + str(len(self.grid_node)) + " by " + str(len(self.grid_node[0])))
        if old_start != grid_start:
            rospy.logwarn("start at coord "+ str(grid_start))
        # rospy.logwarn(self.grid_node)
    
        if ( len(self.grid_node) > grid_start[0]) and (len(self.grid_node[0]) > grid_start[1]):
            self.start = self.grid_node[int(grid_start[0])][int(grid_start[1])]
            
        else:
            self.start = self.instantiate_node(grid_start, 0)
        self.current_state = data.pose

    def make_grid(self, data):
        rospy.loginfo("UPDATING GRID")

        self.gridinfo = data.info
        # rospy.loginfo(self.gridinfo)

        original_height = self.gridinfo.height
        original_width = self.gridinfo.width
        self.size_row = int(self.gridinfo.height/self.resolution_mod)
        self.size_col = int(self.gridinfo.width/self.resolution_mod)
        self.gridinfo.resolution = self.gridinfo.resolution*self.resolution_mod*self.resolution_mod

        self.grid = [[None for i in range(original_width)] for j in range(original_height)]
        self.grid_node = [[None for i in range(self.size_col)] for j in range(self.size_row)]

        for row in range(original_height):
                # rospy.loginfo("read up to row " + str(row))
                self.grid[row] = data.data[int(original_width*row):int(original_width*(row+1)-1)]

        simple_grid = self.simplify_map(self.grid, self.resolution_mod, len(self.grid), len(self.grid[0]))

        for row in range(len(simple_grid)):
            for col in range(len(simple_grid[0])):
                self.grid_node[row][col] = self.instantiate_node((row, col), simple_grid[row][col])
    
        rospy.loginfo("grid made :D")


    def simplify_map(self, grid, resolution, size_row, size_col):

        new_row_size = int(len(grid)/resolution)
        new_col_size = int(len(grid[0])/resolution)

        rospy.loginfo("new height is " + str(new_row_size))
        rospy.loginfo("new width is " + str(new_col_size))

        simple_grid = [[None for i in range(new_col_size)] for j in range(new_row_size)]

        # rospy.logwarn("length of the grid data is :")
        # rospy.loginfo(len(simple_grid))
        # rospy.logwarn("height is :")
        # rospy.loginfo(len(simple_grid[0]))


        for row in range(new_row_size):
            for col in range(new_col_size):
                # rospy.loginfo("made it to :" + str(row) + ", " + str(col))
                simple_grid[row][col] = self.get_value(row, col, grid, resolution)

        return simple_grid
         
    def get_value(self, row, col, grid, resolution):
        new_row = int(row*resolution)
        new_col = int(col*resolution)
        obstacle_threshold = 90
        neighbor_math = []

        for i in range(int(resolution)):
            for j in range(int(resolution)):
                if i != 0 and j !=0:
                    neighbor_math.append([i, j])
        
        if grid[new_row][new_col] > obstacle_threshold:
            return 0
        
        for mod in neighbor_math:
            if grid[new_row+mod[0]][new_col+mod[1]] > obstacle_threshold:
                return 0
        
        return 1
    

    def make_dynamicGrid(self, data):
        self.dynamic_gridinfo = data.info
        self.dynamic_row = self.gridinfo.height
        self.dynamic_col = self.gridinfo.width
        self.dynamic_grid = [[None for i in range(self.dynamic_col)] for j in range(self.dynamic_row)]
        for row in range(self.dynamic_row):
            self.dynamic_grid[row] = data.data[self.dynamic_col*row:self.dynamic_col*(row+1)]

    def grid2dynamic(self, point):
        '''need to align the dynamic sensor occupancy grid with the world occupancy grid
        '''
        row = point[0]*self.gridinfo.resolution + self.gridinfo.origin.position.y
        col = point[1]*self.gridinfo.resolution + self.gridinfo.origin.position.x

        row = (row - self.dynamic_gridinfo.origin.position.y)*self.dynamic_gridinfo.resolution
        col = (col - self.dynamic_gridinfo.origin.position.x)*self.dynamic_gridinfo.resolution

        row = int(row)
        col = int(col)
        # check if the tf coords are within range
        if 0 <= row < self.dynamic_gridinfo.height and 0 <= col < self.dynamic_gridinfo.width:
            return [row, col]
        else:
            return None

    def global2grid(self, point):
        ''' tf from global coordinates to a grid node
        '''
        origin = [self.gridinfo.origin.position.x, self.gridinfo.origin.position.y]
        resolution = self.gridinfo.resolution

        if resolution:
            row = (point[1] - origin[1])/resolution
            col = (point[0] - origin[0])/resolution
        else:
            row = 1000000000
            col = 1000000000
        return [int(row), int(col)]
    
    def grid2global(self, point):
        ''' tf from grid coordinates to a global coords
        '''
        origin = [self.gridinfo.origin.position.x, self.gridinfo.origin.position.y]
        resolution = self.gridinfo.resolution

        if resolution:
            row = point.row*resolution + origin[1]
            col = point.col*resolution + origin[0]
        else: 
            row = 100000000
            col = 100000000
        return [col, row]


    def instantiate_node(self, point, val):
        ''' Instatiate a node given point (x, y) 
            val is the confidence it is an obstacle
        '''
        row, col = point
        if self.dynamic_gridinfo:
            dynamic_point = self.grid2dynamic(point)
            if dynamic_point:
                dynamic_object = (self.dynamic_grid[dynamic_point[0]][dynamic_point[1]] > 50)
            else:
                dynamic_object = False
        else:
            dynamic_object = False  

        if val == 1 or val == -1:
            node = Node(row, col, True,
                        dynamic_object, val)
        else:
            node = Node(row, col, False,
                        dynamic_object, val)
        return node

####### d-star implementation below ####################

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
        # All the 8 neighbors
        for i in range(-1, 2):
            for j in range(-1, 2):
                # Check range
                if row + i < 0 or row + i >= len(self.grid) or \
                        col + j < 0 or col + j >= len(self.grid[0]):
                    continue
                # Do not append the same node
                if i == 0 and j == 0:
                    continue
                if (self.row_min < row < self.row_max) and (self.col_min < col < self.col_max):
                    neighbors.append(self.grid_node[row + i][col + j])
        return neighbors

    def cost(self, node1, node2):
        ''' Euclidean distance from one node to another 

            return:
            distance - Euclidean distance from node 1 to node 2
                       math.inf if one of the node is obstacle
        '''
        # TODO: make the cost change based off the occupancy value,
        # 0-100 based on prob it is obstacle, make cost ? value **4? or 100
        # alter because will come back as obstacle a lot

        # If any of the node is an obstacle
        if node1.is_obs or node2.is_obs:
            return 100000000
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
        while self.open and k_min < h_y and k_min is not 100000000:
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
        obstacle_node.is_obs = True

        # Im going to check if the goal is surrounded
        # kept getting an infinite loop in the path repair
        goal_neighbors = self.get_neighbors(self.goal)
        surrounded = True
        for block in goal_neighbors:
            surrounded = surrounded and block.is_obs

        # if the goal is surrounded the minimum k should be infinity because they all go through an obstacle
        if surrounded:
            print("Goal is blocked off, no path possible")
            return 100000000

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
        neighbors = self.get_neighbors(node)

        for near in neighbors:
            # If neighbor.is_dy_obs == True but neighbor.is_obs == Flase,
            # the neighbor is a new dynamic obstacle
            if near.is_dy_obs and not near.is_obs:
                print("neighbor is new dynamic obstacle")
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

####### end of d-star only functions ######################

    def run(self):
        ''' Run D* algorithm
            Perform the first search from goal to start given the pre-known grid
            Check from start to goal to see if any change happens in the grid, 
            modify the cost and replan in the new map
        '''

        # self.stopRobot()
        rospy.loginfo("trying to find a path")
        # Search from goal to start with the pre-known map
        current_goal = self.goal
        self.insert(self.goal, 0)
        current_state = self.start

        # Process until open set is empty or start is reached
        while self.open and current_goal == self.goal:
            # rospy.loginfo(len(self.open))
            # using self.process_state()
            self.process_state()

        rospy.loginfo("FOUND A PATH!!!!")
        # Visualize the first path if found
        self.get_backpointer_list(self.start)
        rospy.loginfo("path visualized")

        if self.path == []:
            rospy.logwarn("NO PATH! :(")
            print("No path is found")
            return

        # Start from start to goal
        # Update the path if there is any change in the map
        current_node = self.path[0]

        # TODO: Change current state to be robot's location
        while self.current_state is not self.goal and current_goal == self.goal and current_node is not None:
            # Check if any repair needs to be done
            # using self.prepare_repair
            self.prepare_repair(current_node)

            # Replan a path from the current node
            # using self.repair_replan
            self.repair_replan(current_node)

            # Get the new path from the current node
            self.get_backpointer_list(current_node)
            # print("parent of (", current_state.col, ",", current_state.row, ") is (", current_state.parent.col, ",",
            #      current_state.parent.row, ")")

            # for visualizing and move here
            # self.move_robot(current_node)
            current_node = current_node.parent


    def move_robot(self, point):
        ''' move the robot to the given point

            arguments:
            point - Node for the next point in the path
        '''
        current_pos = self.current_state.position
        current_ori = self.current_state.orientation
        orientation_list = [current_ori.x, current_ori.y, current_ori.z, current_ori.w]
        (_, _, yaw) = euler_from_quaternion (orientation_list)
        goal = self.grid2global(point)
        goal_x = goal[1]
        goal_y = goal[0]
        allowable_error = 0.01
        lin_vel_offset = 0.1
        lin_vel_modifyer = 1
        ang_vel_offset = 0.01
        ang_vel_modifyer = 1

        while abs(current_pos.x - goal_x) + abs(current_pos.y - goal_y) > allowable_error:
            heading = math.atan2(goal_y - current_pos.y, goal_x - current_pos.x)
            adjust_heading = yaw - heading
            if adjust_heading > math.pi/2:
                lin_vel = 0
            else:
                lin_vel = lin_vel_modifyer* self.dist(current_pos.x, current_pos.y, goal_x, goal_y) + lin_vel_offset

            ang_vel = adjust_heading*ang_vel_modifyer + ang_vel_offset
            robot_cmd = Twist()
            robot_cmd.linear.x = lin_vel*math.cos(heading)
            robot_cmd.linear.y = lin_vel*math.sin(heading)
            robot_cmd.linear.z = 0.0
            robot_cmd.angular.x = 0.0
            robot_cmd.angular.y = 0.0
            robot_cmd.angular.z = ang_vel
            self.velPub.publish(robot_cmd)
    
    def dist(self, x1, y1, x2, y2):
        return math.sqrt((x1-x2)**2 + (y1-y2**2))

    def get_backpointer_list(self, node):
        ''' Keep tracing back to get the path from a given node to goal '''
        # Assume there is a path from start to goal
        cur_node = node
        self.path = [cur_node]
        point1 = [cur_node.row, cur_node.col]

        self.path_list = []
        while cur_node != self.goal and \
                cur_node != None and \
                not cur_node.is_obs:
            # trace back
            cur_node = cur_node.parent
            if cur_node != None:
                point2 = [cur_node.row, cur_node.col]
                heading = math.atan2(point1[1]-point2[1], point1[0]-point2[0])
                path_point = self.grid2global(cur_node)
                current_pose = self.make_pose(path_point[0], path_point[1], heading)
                point1 = point2
                self.path_list.append(current_pose)
            # add to path
            self.path.append(cur_node)

        # If there is not such a path
        if cur_node != self.goal:
            self.path = []
            self.ros_path.poses = []
        
        if cur_node == self.goal:
            goal_stamped = PoseStamped()
            current_pose = self.goal_pose
            goal_stamped.header.frame_id = "map"
            goal_stamped.header.seq = 0
            goal_stamped.header.stamp.secs = 0
            goal_stamped.header.stamp.nsecs = 0
            goal_stamped.pose = current_pose

            self.path_list.append(goal_stamped)
        
            self.ros_path.poses = self.path_list
            self.ros_path.header.frame_id = "map"
            self.ros_path.header.seq = 0
            self.ros_path.header.stamp.secs = 0
            self.ros_path.header.stamp.nsecs = 0

            self.pathPub.publish(self.ros_path)


    def make_pose(self, x, y, yaw):
        ros_pose = Pose()
        ros_pose.position.x= x
        ros_pose.position.y= y
        ros_pose.position.z= 0
        (xx, yy, zz, ww) = quaternion_from_euler(0, 0, yaw, 'rxyz')
        ros_pose.orientation.x = xx
        ros_pose.orientation.y = yy
        ros_pose.orientation.z = zz
        ros_pose.orientation.w = ww
        stamped = PoseStamped()
        stamped.pose = ros_pose
        stamped.header.frame_id = "map"
        stamped.header.seq = 0
        stamped.header.stamp.secs = 0
        stamped.header.stamp.nsecs = 0
        return stamped


if __name__ == "__main__":

    rospy.init_node('d_star')
   
    # TODO: adjust dynamic grid to be sensor values
    # Search
    d_star = DStar()

    # Run D*
    while not rospy.is_shutdown:
        rospy.spin()
    rospy.spin()
    print("shutting down")
