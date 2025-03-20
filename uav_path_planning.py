# import numpy as np
# import math
# import random
# from queue import PriorityQueue

# class Node:
#     def __init__(self, x, y, z, theta=0, phi=0, parent=None, cost=0):
#         self.x = x
#         self.y = y
#         self.z = z
#         self.theta = theta  # Yaw angle (rotation around z-axis)
#         self.phi = phi      # Pitch angle (rotation around y-axis)
#         self.parent = parent
#         self.cost = cost

#     def __lt__(self, other):
#         return self.cost < other.cost

#     def __eq__(self, other):
#         return (abs(self.x - other.x) < 0.1 and 
#                 abs(self.y - other.y) < 0.1 and 
#                 abs(self.z - other.z) < 0.1)

# class HybridAStar:
#     def __init__(self):
#         self.step_size = 3.0
#         self.turning_radius = 5.0
#         self.theta_steps = 8  # Number of yaw angles
#         self.phi_steps = 4    # Number of pitch angles

#     def get_neighbors(self, node, obstacles):
#         neighbors = []
#         theta_range = np.linspace(-math.pi/4, math.pi/4, self.theta_steps)
#         phi_range = np.linspace(-math.pi/8, math.pi/8, self.phi_steps)  # Smaller range for pitch

#         for theta in theta_range:
#             for phi in phi_range:
#                 new_theta = (node.theta + theta + math.pi) % (2 * math.pi) - math.pi
#                 new_phi = max(min(node.phi + phi, math.pi/2), -math.pi/2)  # Limit pitch

#                 # 3D motion based on spherical coordinates
#                 new_x = node.x + self.step_size * math.cos(new_theta) * math.cos(new_phi)
#                 new_y = node.y + self.step_size * math.sin(new_theta) * math.cos(new_phi)
#                 new_z = node.z + self.step_size * math.sin(new_phi)

#                 if not self.check_collision(new_x, new_y, new_z, obstacles):
#                     new_node = Node(new_x, new_y, new_z, new_theta, new_phi, node, node.cost + self.step_size)
#                     neighbors.append(new_node)
#         return neighbors

#     def heuristic(self, node, goal):
#         return math.sqrt((node.x - goal.x)**2 + (node.y - goal.y)**2 + (node.z - goal.z)**2)

#     def check_collision(self, x, y, z, obstacles):
#         for obs in obstacles:
#             if obs['type'] == 'sphere':
#                 if math.sqrt((x - obs['x'])**2 + (y - obs['y'])**2 + (z - obs['z'])**2) < obs['radius']:
#                     return True
#             elif obs['type'] == 'cuboid':
#                 if (x > obs['x'] and x < obs['x'] + obs['width'] and
#                     y > obs['y'] and y < obs['y'] + obs['height'] and
#                     z > obs['z'] and z < obs['z'] + obs['depth']):
#                     return True
#         return False

#     def find_path(self, start, goal, obstacles, max_iterations=5000):
#         start_node = Node(start[0], start[1], start[2], start[3], start[4])
#         goal_node = Node(goal[0], goal[1], goal[2], goal[3], goal[4])

#         open_set = PriorityQueue()
#         open_set.put((0, start_node))
#         closed_set = set()

#         iteration = 0
#         while not open_set.empty() and iteration < max_iterations:
#             current_cost, current_node = open_set.get()

#             if self.heuristic(current_node, goal_node) < self.step_size:
#                 return self.reconstruct_path(current_node)

#             closed_set.add((round(current_node.x, 2), round(current_node.y, 2), round(current_node.z, 2)))

#             for neighbor in self.get_neighbors(current_node, obstacles):
#                 neighbor_key = (round(neighbor.x, 2), round(neighbor.y, 2), round(neighbor.z, 2))
#                 if neighbor_key in closed_set:
#                     continue

#                 priority = neighbor.cost + self.heuristic(neighbor, goal_node)
#                 open_set.put((priority, neighbor))

#             iteration += 1
#         return None

#     def reconstruct_path(self, node):
#         path = []
#         current = node
#         while current is not None:
#             path.append((current.x, current.y, current.z))
#             current = current.parent
#         return path[::-1]

# class RRTPlanner:
#     def __init__(self):
#         self.step_size = 3.0
#         self.goal_sample_rate = 0.1
#         self.max_iterations = 5000

#     def sample_point(self, bounds, goal, obstacles):
#         if random.random() < self.goal_sample_rate:
#             return goal.x, goal.y, goal.z

#         while True:
#             x = random.uniform(bounds['min_x'], bounds['max_x'])
#             y = random.uniform(bounds['min_y'], bounds['max_y'])
#             z = random.uniform(bounds['min_z'], bounds['max_z'])
#             if not self.check_collision(x, y, z, obstacles):
#                 return x, y, z

#     def find_nearest(self, nodes, point):
#         distances = [(node, math.sqrt((node.x - point[0])**2 + (node.y - point[1])**2 + (node.z - point[2])**2))
#                      for node in nodes]
#         return min(distances, key=lambda x: x[1])[0]

#     def steer(self, from_node, to_point):
#         dx = to_point[0] - from_node.x
#         dy = to_point[1] - from_node.y
#         dz = to_point[2] - from_node.z
#         distance = math.sqrt(dx**2 + dy**2 + dz**2)

#         if distance < self.step_size:
#             new_x, new_y, new_z = to_point[0], to_point[1], to_point[2]
#         else:
#             theta = math.atan2(dy, dx)
#             phi = math.asin(dz / distance)
#             new_x = from_node.x + self.step_size * math.cos(theta) * math.cos(phi)
#             new_y = from_node.y + self.step_size * math.sin(theta) * math.cos(phi)
#             new_z = from_node.z + self.step_size * math.sin(phi)

#         return Node(new_x, new_y, new_z, parent=from_node)

#     def check_collision(self, x, y, z, obstacles):
#         for obs in obstacles:
#             if obs['type'] == 'sphere':
#                 if math.sqrt((x - obs['x'])**2 + (y - obs['y'])**2 + (z - obs['z'])**2) < obs['radius']:
#                     return True
#             elif obs['type'] == 'cuboid':
#                 if (x > obs['x'] and x < obs['x'] + obs['width'] and
#                     y > obs['y'] and y < obs['y'] + obs['height'] and
#                     z > obs['z'] and z < obs['z'] + obs['depth']):
#                     return True
#         return False

#     def find_path(self, start, goal, obstacles, bounds):
#         start_node = Node(start[0], start[1], start[2])
#         goal_node = Node(goal[0], goal[1], goal[2])
#         nodes = [start_node]

#         for _ in range(self.max_iterations):
#             rnd_point = self.sample_point(bounds, goal_node, obstacles)
#             nearest_node = self.find_nearest(nodes, rnd_point)
#             new_node = self.steer(nearest_node, rnd_point)

#             if not self.check_collision(new_node.x, new_node.y, new_node.z, obstacles):
#                 nodes.append(new_node)
#                 if self.heuristic(new_node, goal_node) < self.step_size:
#                     goal_node.parent = new_node
#                     return self.reconstruct_path(goal_node)
#         return None

#     def heuristic(self, node, goal):
#         return math.sqrt((node.x - goal.x)**2 + (node.y - goal.y)**2 + (node.z - goal.z)**2)

#     def reconstruct_path(self, node):
#         path = []
#         current = node
#         while current is not None:
#             path.append((current.x, current.y, current.z))
#             current = current.parent
#         return path[::-1]

# class CombinedPathPlanner:
#     def __init__(self):
#         self.hybrid_astar = HybridAStar()
#         self.rrt = RRTPlanner()

#     def plan_path(self, start, goal, obstacles, bounds):
#         print("Attempting Hybrid A* path planning...")
#         path = self.hybrid_astar.find_path(start, goal, obstacles)

#         if path is None:
#             print("Hybrid A* failed. Switching to RRT...")
#             path = self.rrt.find_path(start[:3], goal[:3], obstacles, bounds)

#         return path

# # For standalone testing (optional)
# def main():
#     bounds = {'min_x': 0, 'max_x': 100, 'min_y': 0, 'max_y': 100, 'min_z': 0, 'max_z': 50}
#     obstacles = [
#         {'type': 'sphere', 'x': 20, 'y': 20, 'z': 10, 'radius': 5},
#         {'type': 'cuboid', 'x': 60, 'y': 60, 'z': 0, 'width': 10, 'height': 20, 'depth': 15}
#     ]
#     start = (10, 10, 0, 0, 0)  # [x, y, z, theta, phi]
#     goal = (90, 90, 40, 0, 0)
#     planner = CombinedPathPlanner()
#     path = planner.plan_path(start, goal, obstacles, bounds)
#     if path:
#         print("Path found:", path)
#     else:
#         print("No path found!")

# if __name__ == "__main__":
#     main()


import numpy as np
import math
import random
from queue import PriorityQueue

class Node:
    def __init__(self, x, y, z, theta=0, phi=0, parent=None, cost=0):
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta
        self.phi = phi
        self.parent = parent
        self.cost = cost

    def __lt__(self, other):
        return self.cost < other.cost

    def __eq__(self, other):
        return (abs(self.x - other.x) < 0.1 and 
                abs(self.y - other.y) < 0.1 and 
                abs(self.z - other.z) < 0.1)

class HybridAStar:
    def __init__(self):
        self.step_size = 5.0  # Increased for faster exploration
        self.turning_radius = 5.0
        self.theta_steps = 8
        self.phi_steps = 4

    def get_neighbors(self, node, obstacles):
        neighbors = []
        theta_range = np.linspace(-math.pi/4, math.pi/4, self.theta_steps)
        phi_range = np.linspace(-math.pi/8, math.pi/8, self.phi_steps)

        for theta in theta_range:
            for phi in phi_range:
                new_theta = (node.theta + theta + math.pi) % (2 * math.pi) - math.pi
                new_phi = max(min(node.phi + phi, math.pi/2), -math.pi/2)

                new_x = node.x + self.step_size * math.cos(new_theta) * math.cos(new_phi)
                new_y = node.y + self.step_size * math.sin(new_theta) * math.cos(new_phi)
                new_z = node.z + self.step_size * math.sin(new_phi)

                if not self.check_collision(new_x, new_y, new_z, obstacles):
                    new_node = Node(new_x, new_y, new_z, new_theta, new_phi, node, node.cost + self.step_size)
                    neighbors.append(new_node)
        return neighbors

    def heuristic(self, node, goal):
        return math.sqrt((node.x - goal.x)**2 + (node.y - goal.y)**2 + (node.z - goal.z)**2)

    def check_collision(self, x, y, z, obstacles):
        for obs in obstacles:
            if obs['type'] == 'sphere':
                if math.sqrt((x - obs['x'])**2 + (y - obs['y'])**2 + (z - obs['z'])**2) < obs['radius']:
                    return True
            elif obs['type'] == 'cuboid':
                if (x > obs['x'] and x < obs['x'] + obs['width'] and
                    y > obs['y'] and y < obs['y'] + obs['height'] and
                    z > obs['z'] and z < obs['z'] + obs['depth']):
                    return True
        return False

    def find_path(self, start, goal, obstacles, max_iterations=10000):  # Increased iterations
        start_node = Node(start[0], start[1], start[2], start[3], start[4])
        goal_node = Node(goal[0], goal[1], goal[2], goal[3], goal[4])

        open_set = PriorityQueue()
        open_set.put((0, start_node))
        closed_set = set()

        iteration = 0
        while not open_set.empty() and iteration < max_iterations:
            current_cost, current_node = open_set.get()

            if self.heuristic(current_node, goal_node) < self.step_size:
                print("Hybrid A* found a path!")
                return self.reconstruct_path(current_node)

            closed_set.add((round(current_node.x, 2), round(current_node.y, 2), round(current_node.z, 2)))

            for neighbor in self.get_neighbors(current_node, obstacles):
                neighbor_key = (round(neighbor.x, 2), round(neighbor.y, 2), round(neighbor.z, 2))
                if neighbor_key in closed_set:
                    continue

                priority = neighbor.cost + self.heuristic(neighbor, goal_node)
                open_set.put((priority, neighbor))

            iteration += 1
        print("Hybrid A* failed to find a path.")
        return None

    def reconstruct_path(self, node):
        path = []
        current = node
        while current is not None:
            path.append((current.x, current.y, current.z))
            current = current.parent
        return path[::-1]

class RRTPlanner:
    def __init__(self):
        self.step_size = 5.0  # Increased for faster exploration
        self.goal_sample_rate = 0.2  # Increased to bias towards goal
        self.max_iterations = 10000  # Increased iterations

    def sample_point(self, bounds, goal, obstacles):
        if random.random() < self.goal_sample_rate:
            return goal.x, goal.y, goal.z

        while True:
            x = random.uniform(bounds['min_x'], bounds['max_x'])
            y = random.uniform(bounds['min_y'], bounds['max_y'])
            z = random.uniform(bounds['min_z'], bounds['max_z'])
            if not self.check_collision(x, y, z, obstacles):
                return x, y, z

    def find_nearest(self, nodes, point):
        distances = [(node, math.sqrt((node.x - point[0])**2 + (node.y - point[1])**2 + (node.z - point[2])**2))
                     for node in nodes]
        return min(distances, key=lambda x: x[1])[0]

    def steer(self, from_node, to_point):
        dx = to_point[0] - from_node.x
        dy = to_point[1] - from_node.y
        dz = to_point[2] - from_node.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        if distance < self.step_size:
            new_x, new_y, new_z = to_point[0], to_point[1], to_point[2]
        else:
            theta = math.atan2(dy, dx)
            phi = math.asin(dz / distance)
            new_x = from_node.x + self.step_size * math.cos(theta) * math.cos(phi)
            new_y = from_node.y + self.step_size * math.sin(theta) * math.cos(phi)
            new_z = from_node.z + self.step_size * math.sin(phi)

        return Node(new_x, new_y, new_z, parent=from_node)

    def check_collision(self, x, y, z, obstacles):
        for obs in obstacles:
            if obs['type'] == 'sphere':
                if math.sqrt((x - obs['x'])**2 + (y - obs['y'])**2 + (z - obs['z'])**2) < obs['radius']:
                    return True
            elif obs['type'] == 'cuboid':
                if (x > obs['x'] and x < obs['x'] + obs['width'] and
                    y > obs['y'] and y < obs['y'] + obs['height'] and
                    z > obs['z'] and z < obs['z'] + obs['depth']):
                    return True
        return False

    def find_path(self, start, goal, obstacles, bounds):
        start_node = Node(start[0], start[1], start[2])
        goal_node = Node(goal[0], goal[1], goal[2])
        nodes = [start_node]

        for _ in range(self.max_iterations):
            rnd_point = self.sample_point(bounds, goal_node, obstacles)
            nearest_node = self.find_nearest(nodes, rnd_point)
            new_node = self.steer(nearest_node, rnd_point)

            if not self.check_collision(new_node.x, new_node.y, new_node.z, obstacles):
                nodes.append(new_node)
                if self.heuristic(new_node, goal_node) < self.step_size:
                    goal_node.parent = new_node
                    print("RRT found a path!")
                    return self.reconstruct_path(goal_node)
        print("RRT failed to find a path.")
        return None

    def heuristic(self, node, goal):
        return math.sqrt((node.x - goal.x)**2 + (node.y - goal.y)**2 + (node.z - goal.z)**2)

    def reconstruct_path(self, node):
        path = []
        current = node
        while current is not None:
            path.append((current.x, current.y, current.z))
            current = current.parent
        return path[::-1]

class CombinedPathPlanner:
    def __init__(self):
        self.hybrid_astar = HybridAStar()
        self.rrt = RRTPlanner()

    def plan_path(self, start, goal, obstacles, bounds):
        print("Attempting Hybrid A* path planning...")
        path = self.hybrid_astar.find_path(start, goal, obstacles)

        if path is None:
            print("Hybrid A* failed. Switching to RRT...")
            path = self.rrt.find_path(start[:3], goal[:3], obstacles, bounds)

        return path

def main():
    bounds = {'min_x': 0, 'max_x': 800, 'min_y': 0, 'max_y': 600, 'min_z': -100, 'max_z': 100}
    obstacles = [
        {'type': 'sphere', 'x': 200, 'y': 200, 'z': 0, 'radius': 30},  # Reduced radius
        {'type': 'cuboid', 'x': 400, 'y': 300, 'z': 50, 'width': 80, 'height': 150, 'depth': 40}  # Reduced size
    ]
    start = (100, 100, 0, 0, 0)
    goal = (700, 500, 50, 0, 0)
    planner = CombinedPathPlanner()
    path = planner.plan_path(start, goal, obstacles, bounds)
    if path:
        print("Path found:", path)
    else:
        print("No path found!")

if __name__ == "__main__":
    main()