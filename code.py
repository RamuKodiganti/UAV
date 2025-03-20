# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# import matplotlib.animation as animation
# from uav_path_planning import CombinedPathPlanner

# # Move dynamic obstacles in 3D
# def move_dynamic_obstacles(dynamic_obstacles, bounds):
#     for obs in dynamic_obstacles:
#         obs['x'] += obs['speed_x']
#         obs['y'] += obs['speed_y']
#         obs['z'] += obs['speed_z']
#         if (obs['x'] - obs['radius'] < bounds['min_x'] or obs['x'] + obs['radius'] > bounds['max_x']):
#             obs['speed_x'] *= -1
#         if (obs['y'] - obs['radius'] < bounds['min_y'] or obs['y'] + obs['radius'] > bounds['max_y']):
#             obs['speed_y'] *= -1
#         if (obs['z'] - obs['radius'] < bounds['min_z'] or obs['z'] + obs['radius'] > bounds['max_z']):
#             obs['speed_z'] *= -1

# def visualize_path(path, obstacles, dynamic_obstacles, start, goal, bounds):
#     fig = plt.figure(figsize=(10, 10))
#     ax = fig.add_subplot(111, projection='3d')

#     def init():
#         ax.clear()
#         for obs in obstacles:
#             if obs['type'] == 'sphere':
#                 u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
#                 x = obs['x'] + obs['radius'] * np.cos(u) * np.sin(v)
#                 y = obs['y'] + obs['radius'] * np.sin(u) * np.sin(v)
#                 z = obs['z'] + obs['radius'] * np.cos(v)
#                 ax.plot_surface(x, y, z, color='red', alpha=0.5)
#             elif obs['type'] == 'cuboid':
#                 r = [obs['x'], obs['x'] + obs['width']]
#                 s = [obs['y'], obs['y'] + obs['height']]
#                 t = [obs['z'], obs['z'] + obs['depth']]
#                 for i in range(2):
#                     for j in range(2):
#                         ax.plot3D([r[i], r[i]], [s[j], s[j]], [t[0], t[1]], 'r')

#         ax.scatter([start[0]], [start[1]], [start[2]], color='g', label='Start', s=100)
#         ax.scatter([goal[0]], [goal[1]], [goal[2]], color='r', label='Goal', s=100)
#         ax.set_xlim(bounds['min_x'], bounds['max_x'])
#         ax.set_ylim(bounds['min_y'], bounds['max_y'])
#         ax.set_zlim(bounds['min_z'], bounds['max_z'])
#         ax.set_xlabel('X')
#         ax.set_ylabel('Y')
#         ax.set_zlabel('Z')
#         ax.legend()
#         return []

#     def animate(i):
#         move_dynamic_obstacles(dynamic_obstacles, bounds)
#         ax.clear()
#         init()  # Redraw static elements

#         # Plot dynamic obstacles
#         for obs in dynamic_obstacles:
#             u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
#             x = obs['x'] + obs['radius'] * np.cos(u) * np.sin(v)
#             y = obs['y'] + obs['radius'] * np.sin(u) * np.sin(v)
#             z = obs['z'] + obs['radius'] * np.cos(v)
#             ax.plot_surface(x, y, z, color='orange', alpha=0.7)

#         # Plot path
#         if path:
#             path = np.array(path)
#             ax.plot(path[:, 0], path[:, 1], path[:, 2], 'b-', label='Path')
#         ax.legend()
#         return []

#     ani = animation.FuncAnimation(fig, animate, init_func=init, frames=200, blit=False, repeat=True)
#     plt.show()

# def main():
#     bounds = {'min_x': 0, 'max_x': 100, 'min_y': 0, 'max_y': 100, 'min_z': 0, 'max_z': 50}
#     static_obstacles = [
#         {'type': 'sphere', 'x': 20, 'y': 20, 'z': 10, 'radius': 5},
#         {'type': 'cuboid', 'x': 60, 'y': 60, 'z': 0, 'width': 10, 'height': 20, 'depth': 15}
#     ]
#     dynamic_obstacles = [
#         {'type': 'sphere', 'x': 40, 'y': 50, 'z': 20, 'radius': 8, 'speed_x': 0.5, 'speed_y': 0.3, 'speed_z': 0.2}
#     ]
#     obstacles = static_obstacles + dynamic_obstacles
#     start = (10, 10, 0, 0, 0)  # [x, y, z, theta, phi]
#     goal = (90, 90, 40, 0, 0)

#     planner = CombinedPathPlanner()
#     path = planner.plan_path(start, goal, obstacles, bounds)

#     if path:
#         print("Path found successfully!")
#         visualize_path(path, static_obstacles, dynamic_obstacles, start, goal, bounds)
#     else:
#         print("No path found!")

# if __name__ == "__main__":
#     main()






import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from uav_path_planning import CombinedPathPlanner

def move_dynamic_obstacles(dynamic_obstacles, bounds):
    for obs in dynamic_obstacles:
        obs['x'] += obs['speed_x']
        obs['y'] += obs['speed_y']
        obs['z'] += obs['speed_z']
        if (obs['x'] - obs['radius'] < bounds['min_x'] or obs['x'] + obs['radius'] > bounds['max_x']):
            obs['speed_x'] *= -1
        if (obs['y'] - obs['radius'] < bounds['min_y'] or obs['y'] + obs['radius'] > bounds['max_y']):
            obs['speed_y'] *= -1
        if (obs['z'] - obs['radius'] < bounds['min_z'] or obs['z'] + obs['radius'] > bounds['max_z']):
            obs['speed_z'] *= -1

def visualize_path(path, obstacles, dynamic_obstacles, start, goal, bounds):
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')

    def init():
        ax.clear()
        for obs in obstacles:
            if obs['type'] == 'sphere':
                u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
                x = obs['x'] + obs['radius'] * np.cos(u) * np.sin(v)
                y = obs['y'] + obs['radius'] * np.sin(u) * np.sin(v)
                z = obs['z'] + obs['radius'] * np.cos(v)
                ax.plot_surface(x, y, z, color='red', alpha=0.5)
            elif obs['type'] == 'cuboid':
                r = [obs['x'], obs['x'] + obs['width']]
                s = [obs['y'], obs['y'] + obs['height']]
                t = [obs['z'], obs['z'] + obs['depth']]
                for i in range(2):
                    for j in range(2):
                        ax.plot3D([r[i], r[i]], [s[j], s[j]], [t[0], t[1]], 'r')

        ax.scatter([start[0]], [start[1]], [start[2]], color='g', label='Start', s=100)
        ax.scatter([goal[0]], [goal[1]], [goal[2]], color='r', label='Goal', s=100)
        ax.set_xlim(bounds['min_x'], bounds['max_x'])
        ax.set_ylim(bounds['min_y'], bounds['max_y'])
        ax.set_zlim(bounds['min_z'], bounds['max_z'])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        return []

    def animate(i):
        move_dynamic_obstacles(dynamic_obstacles, bounds)
        ax.clear()
        init()

        for obs in dynamic_obstacles:
            u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
            x = obs['x'] + obs['radius'] * np.cos(u) * np.sin(v)
            y = obs['y'] + obs['radius'] * np.sin(u) * np.sin(v)
            z = obs['z'] + obs['radius'] * np.cos(v)
            ax.plot_surface(x, y, z, color='orange', alpha=0.7)

        if path:
            path = np.array(path)
            ax.plot(path[:, 0], path[:, 1], path[:, 2], 'b-', label='Path')
        ax.legend()
        return []

    ani = animation.FuncAnimation(fig, animate, init_func=init, frames=200, blit=False, repeat=True)
    plt.show()

def main():
    bounds = {'min_x': 0, 'max_x': 800, 'min_y': 0, 'max_y': 600, 'min_z': -100, 'max_z': 100}
    static_obstacles = [
        {'type': 'sphere', 'x': 200, 'y': 200, 'z': 0, 'radius': 30},
        {'type': 'cuboid', 'x': 400, 'y': 300, 'z': 50, 'width': 80, 'height': 150, 'depth': 40}
    ]
    dynamic_obstacles = []  # Temporarily disabled for debugging
    obstacles = static_obstacles + dynamic_obstacles
    start = (100, 100, 0, 0, 0)
    goal = (700, 500, 50, 0, 0)

    planner = CombinedPathPlanner()
    path = planner.plan_path(start, goal, obstacles, bounds)

    if path:
        print("Path found successfully!")
        visualize_path(path, static_obstacles, dynamic_obstacles, start, goal, bounds)
    else:
        print("No path found!")

if __name__ == "__main__":
    main()