import csv
# import os
from search import DStar


# Load map, start and goal point.
def load_map(file_path):
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
    return grid, start, goal


if __name__ == "__main__":
    # Load the map
    # print(os.getcwd())
    scale = 10
    cspace = False
    show_static_map = True

    if cspace:
        grid, start, goal = load_map('src/PathPlanning/path_planning/src/2d_imp/maps/hospital'+ str(scale) + '.csv')
        dynamic_grid, _, _ = load_map('src/PathPlanning/path_planning/src/2d_imp/maps/hospital'+ str(scale) + '.csv')
    else:
        grid, start, goal = load_map('src/PathPlanning/path_planning/src/2d_imp/maps/hospital'+ str(scale) + '_nocspace.csv')
        dynamic_grid, _, _ = load_map('src/PathPlanning/path_planning/src/2d_imp/maps/hospital'+ str(scale) + '_nocspace.csv')

    grid, _, _ = load_map('src/PathPlanning/path_planning/src/2d_imp/maps/dynamic_maps/hospital10_nocspace_dynamic34.csv')

    # smaller maps
    #grid, start, goal = load_map('src/PathPlanning/path_planning/src/2d_imp/maps/map1.csv')
    #dynamic_grid, _, _ = load_map('src/PathPlanning/path_planning/src/2d_imp/maps/unknown_map1.csv')
    is_dynamic = True
    # Search
    d_star = DStar(grid, dynamic_grid, start, goal, is_dynamic)
    print("initialized, making map")
        
    # Visualize the map
    if show_static_map:
        d_star.draw_path(grid, "static map x")
        print("map made, running D*")
    else:
        print("running D*")
        
    # Run D*
    # d_star.run()
