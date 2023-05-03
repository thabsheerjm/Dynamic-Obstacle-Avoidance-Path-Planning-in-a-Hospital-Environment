import csv
import math

def convert_ocgrid2coord():
    resolution = 0.1  # 10 cm between two adjacent cells

    occupied_coords = []

    with open('./hospital0.csv', 'r') as f:
        reader = csv.reader(f)
        occupancy_grid = [[int(val) for val in row] for row in reader]

    for i in range(len(occupancy_grid)):
        for j in range(len(occupancy_grid[0])):
            if occupancy_grid[i][j] == 0:  # cell is occupied
                x = i * resolution  # x-coordinate of the cell center
                y = j * resolution  # y-coordinate of the cell center
                occupied_coords.append((x, y))

    return occupied_coords,occupancy_grid

def normal(coordinates):
    temp = []
    for x in coordinates:
        temp.append(x[0])
    min_x,max_x = min(temp),max(temp)
    temp = []
    for y in coordinates:
        temp.append(y[0])
    min_y,max_y = min(temp),max(temp)

    return math.sqrt((max_x-min_x)**2+(max_y-min_y)**2), max_x-min_x, max_y-min_y,  (max_x, min_x, max_y,min_y)

def transform_coordinates(coordinates, width, height):
    center_x = width/2 
    center_y = height/2 

    transformed_coordinates = []

    for x, y in coordinates:
        new_x = x - center_x
        new_y = y - center_y
        transformed_coordinates.append((new_x, new_y))

    return transformed_coordinates


if __name__ == '__main__':
    coordinates,_ = convert_ocgrid2coord()
    
