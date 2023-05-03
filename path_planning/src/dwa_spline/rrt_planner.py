import numpy as np
import random

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def euclidean_distance(a, b):
    return np.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2)

def collision_free(new_node, nearest_node, obstacles, clearance=1):
    for obs in obstacles:
        obs_node = Node(obs[0], obs[1])
        if euclidean_distance(new_node, obs_node) < clearance:
            return False
    return True

def find_nearest_node(node_list, rnd_node):
    nearest_node = node_list[0]
    min_dist = euclidean_distance(nearest_node, rnd_node)

    for node in node_list:
        dist = euclidean_distance(node, rnd_node)
        if dist < min_dist:
            min_dist = dist
            nearest_node = node

    return nearest_node

def rrt_planner(start, goal, obstacles, iterations=5000, goal_tolerance=2, expansion_step=0.5,r =100):
    start_node = Node(start[0], start[1])
    goal_node = Node(goal[0], goal[1])

    node_list = [start_node]

    for _ in range(iterations):
        rnd_x = random.uniform(0,r)  
        rnd_y = random.uniform(0, r)
        rnd_node = Node(rnd_x, rnd_y)

        nearest_node = find_nearest_node(node_list, rnd_node)
        distance = euclidean_distance(nearest_node, rnd_node)

        new_node = Node(nearest_node.x + (rnd_node.x - nearest_node.x) * (expansion_step / distance),
                        nearest_node.y + (rnd_node.y - nearest_node.y) * (expansion_step / distance))
        new_node.parent = nearest_node

        if collision_free(new_node, nearest_node, obstacles):
            node_list.append(new_node)

        if euclidean_distance(new_node, goal_node) < goal_tolerance:  
            goal_node.parent = new_node
            break

    path = []
    node = goal_node

    while node is not None:
        path.append((node.x, node.y))
        node = node.parent

    return path[::-1]

