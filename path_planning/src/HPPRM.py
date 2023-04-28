import numpy as np
import matplotlib.pyplot as plt
import cv2
from decimal import Decimal
import csv
from sklearn.neighbors import NearestNeighbors
from .Dijkstra import Graph, dijkstra, to_array

class APF:
    def __init__(self, influence_coefficient, repulsion_range):
        self.influence_coefficient = influence_coefficient
        self.repulsion_range = repulsion_range

    def repulsive_potential(self, x, y, ox, oy):
        id = -1
        dmin = float("inf")
        for i, _ in enumerate(ox):
            d = np.hypot(x - ox[i], y - oy[i])
            if dmin >= d:
                dmin = d
                id = i

        # distance between distance between current position q and obstacle q0
        dq = np.hypot(x - ox[id], y - oy[id])

        if dq <= self.repulsion_range:
            if dq <= 0.1:
                dq = 0.1

            return 0.5 * self.influence_coefficient * (
                1.0 / dq - 1.0 / self.repulsion_range) ** 2
        else:
            return 0.0

    def sampling(self, map):
        sampling_points = 0
        obs_x, obs_y, white_reg, samples_obs_region, samples_open_region = [], [], [], [], []
        im_rep = map.copy()
        for i in range(im_rep.shape[0]):
            for j in range(im_rep.shape[1]):
                if im_rep[i][j] == 0:
                    obs_x.append(i)
                    obs_y.append(j)
                else:
                    white_reg.append([i, j])

        for i, j in enumerate(white_reg):
            rep = self.repulsive_potential(
                j[0], j[1], obs_x, obs_y)
            if rep > 1:
                im_rep[j[0], j[1]] = 127
                samples_obs_region.append(j)
                sampling_points = sampling_points + 1
            else:
                samples_open_region.append(j)
        return sampling_points, samples_obs_region, samples_open_region

class PRM:
    def __init__(
            self,
            current,
            goal,
            sampling_points,
            samples_obs_region,
            samples_open_region):
        self.sampling_coords = np.array([])
        self.current = np.array(current)
        self.goal = np.array(goal)
        self.sampling_points = sampling_points
        self.samples_open_region = samples_open_region
        self.samples_obs_region = samples_obs_region
        self.graph = Graph()
        self.pathFound = False

    def prm(self):
        print("Solving...")
        while(not self.pathFound):
            self.samplingCoords()
            self.checkForObstacle()
            self.nearestNeighbour()
            self.path()
            self.sampling_coords = np.array([])
            self.graph = Graph()

        plt.savefig("./results/final_path.png")
        plt.show()

    def samplingCoords(self):
        no_of_samples_obs = int(self.sampling_points / 5)
        no_of_samples_wr = int(no_of_samples_obs * 2)
        sample_obs_idx = np.random.randint(
            0, len(self.samples_obs_region), size=no_of_samples_obs)
        sample_wr_idx = np.random.randint(
            0, len(self.samples_open_region),
            size=no_of_samples_wr)
        list_use = []
        for i, j in enumerate(sample_obs_idx):
            list_use.append(self.samples_obs_region[j])
        for i, j in enumerate(sample_wr_idx):
            list_use.append(self.samples_open_region[j])

        self.sampling_coords = np.array(list_use)
        self.current = self.current.reshape(1, 2)
        self.goal = self.goal.reshape(1, 2)
        self.sampling_coords = np.concatenate(
            (self.sampling_coords, self.current, self.goal), axis=0)

    def checkForObstacle(self):
        collision = False
        self.collisionFreePoints = np.array([])
        for point in self.sampling_coords:
            if(self.collisionFreePoints.size == 0):
                self.collisionFreePoints = point
            else:
                self.collisionFreePoints = np.vstack(
                    [self.collisionFreePoints, point])
        self.plotSamplingPoints(self.collisionFreePoints)

    def nearestNeighbour(self, k=5):
        X = self.collisionFreePoints
        knn = NearestNeighbors(n_neighbors=k)
        knn.fit(X)
        distances, indices = knn.kneighbors(X)
        self.collisionFreePaths = np.empty((1, 2), int)

        for i, strt in enumerate(X):
            for j, neighbour in enumerate(X[indices[i][1:]]):
                start_line = strt
                end_line = neighbour
                node_idx = str(self.nodeIdx(strt))
                n_node_idx = str(self.nodeIdx(neighbour))
                self.graph.add_node(node_idx)
                self.graph.add_edge(node_idx, n_node_idx, distances[i, j + 1])
                x = [strt[0], neighbour[0]]
                y = [strt[1], neighbour[1]]
                plt.plot(x, y, c="red", linewidth=0.3)

    def path(self):
        self.startNode = str(self.nodeIdx(self.current))
        self.endNode = str(self.nodeIdx(self.goal))

        dist, prev = dijkstra(self.graph, self.startNode)

        end_path = to_array(prev, self.endNode)

        if(len(end_path) > 1):
            self.pathFound = True
        else:
            return

        plot_plots = [(self.findPoints(path))
                      for path in end_path]

        x = [int(item[0]) for item in plot_plots]
        y = [int(item[1]) for item in plot_plots]
        plt.plot(x, y, c="black", linewidth=3.5)

        pointsToEnd = [self.findPoints(path)
                       for path in end_path]
        print("Path Found !!!")
        print("Coordinates saved in results/data.csv")
        CSVFile_path = "./results/data.csv"
        header = [['x', 'y']]
        with open(CSVFile_path, 'w', encoding='UTF8', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(header)
            writer.writerows(pointsToEnd)

    def nodeIdx(self, pt):
        return np.where((self.collisionFreePoints == pt).all(axis=1))[0][0]

    def findPoints(self, n):
        return self.collisionFreePoints[int(n)]

    def plotSamplingPoints(self, points):
        x = [item[0] for item in points]
        y = [item[1] for item in points]
        plt.scatter(x, y, c="black", s=1)

class Utils:
    def __init__(
            self,
            current,
            destination,
            occupancy_grid_map):
        self.current = np.array(current)
        self.destination = np.array(destination)
        self.occupancy_grid_map = occupancy_grid_map
        self.map_inflated = None

    def getInflatedMap(self):
        self.initializeMap()
        return self.map_inflated

    def drawMap(self):
        fig = plt.figure(figsize=(8, 8))
        currentAxis = plt.gca()
        curr = self.current
        dest = self.destination
        plt.scatter(curr[0], curr[1], s=200, c='green')
        plt.scatter(dest[0], dest[1], s=200, c='green')
        fig.canvas.draw()

    def initializeMap(self):
        thresh = 170
        # gImg = cv2.cvtColor(self.occupancy_grid_map , cv2.COLOR_BGR2GRAY )
        im_bw = cv2.adaptiveThreshold(
            self.occupancy_grid_map,
            thresh,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, 31,1)[1]
        kernel = np.ones((3, 3), np.uint8)
        im_bw = cv2.erode(im_bw, kernel, iterations=1)
        im_bw = cv2.copyMakeBorder(
            im_bw, 3, 3, 3, 3, cv2.BORDER_CONSTANT, value=0)
        self.map_inflated = cv2.rotate(im_bw, cv2.ROTATE_90_CLOCKWISE)