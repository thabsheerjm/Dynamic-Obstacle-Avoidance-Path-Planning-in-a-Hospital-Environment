#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist, Pose
from nav_msgs.msg import OccupancyGrid, Path, MapMetaData
import csv
import numpy as np
from PIL import Image as im



class MapMaker:
    def __init__(self, resolution_mod):
        self.resolution_mod = resolution_mod
        # rospy.Subscriber("/gopher_presence/move_base/global_costmap/costmap", OccupancyGrid, self.make_grid)
        rospy.Subscriber("/map", OccupancyGrid, self.make_grid)


    def make_grid(self, data):
            rospy.loginfo("UPDATING GRID")

            gridinfo = data.info
            

            original_height = gridinfo.height
            original_width = gridinfo.width
            size_row = int(gridinfo.height/self.resolution_mod)
            size_col = int(gridinfo.width/self.resolution_mod)
            gridinfo.resolution = gridinfo.resolution*self.resolution_mod*self.resolution_mod

            grid = [[None for i in range(original_width)] for j in range(original_height)]
            #rospy.logwarn("length of the grid data is :")
            #rospy.loginfo(len(data.data))
            #rospy.logwarn("attempting to read up to value :")
            #rospy.loginfo((original_height)*original_width)
            rospy.loginfo("original height is " + str(original_height))
            rospy.loginfo("original width is " + str(original_width))

            for row in range(original_height):
                # rospy.loginfo("read up to row " + str(row))
                grid[row] = data.data[int(original_width*row):int(original_width*(row+1)-1)]
            
            rospy.loginfo("grid height is " + str(len(grid)))
            rospy.loginfo("grid width is " + str(len(grid[0])))

            simple_grid = self.simplify_map(grid, self.resolution_mod, len(grid), len(grid[0]))
            
            rospy.loginfo("simple height is " + str(len(simple_grid)))
            rospy.loginfo("simple width is " + str(len(simple_grid[0])))

            rospy.loginfo("grid made :D")
            rospy.loginfo("outputing to csv")

            with open("src/PathPlanning/path_planning/src/2d_imp/hospital" + str(self.resolution_mod) +".csv","w+") as my_csv:
                csvWriter = csv.writer(my_csv,delimiter=',')
                start = [455, 575] # for a resolution of 1
                start = [int(start[0]/self.resolution_mod), int(start[1]/self.resolution_mod)]
                goal = [506, 560]
                goal = [int(goal[0]/self.resolution_mod), int(goal[1]/self.resolution_mod)]
                csvWriter.writerow(["start", str(start[0]), str(start[1])])
                csvWriter.writerow(["goal", str(goal[0]), str(goal[1])])
                for ro in simple_grid:
                    csvWriter.writerow(ro)
            
            rospy.loginfo("MAP MADE!")
    
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


if __name__ == "__main__":
    rospy.init_node('cartographer', anonymous=True)
    rospy.logwarn("Cartographer is setting up his station")
    # Load the map
    map_scale = 1
    rospy.loginfo("making a map that is 1:"+ str(map_scale*map_scale))
    rospy.loginfo("(1 csv grid block is " + str(map_scale*map_scale) +  " occupancy grid blocks)")
    cartographer = MapMaker(map_scale)

    rospy.spin()

    # Run D*
    # d_star.run()
    print("artographer going home, Bye!")
