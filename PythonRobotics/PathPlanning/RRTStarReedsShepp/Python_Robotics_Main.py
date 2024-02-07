#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from rrt_star_reeds_shepp import RRTStarReedsShepp


def main():
    # Define the start and goal positions
    start = (0,0,0)         # start:Start Position [x,y]
    goal = (10,10,np.pi)          # goal:Goal Position [x,y]
    obs_list = []           # obstacleList:obstacle Positions [[x,y,size],...]
    obs_list.append((3,3,1))
    obs_list.append((2,3,1))
    obs_list.append((1,3,1))
    rand_area = [-10, 10]    # randArea:Random Sampling Area [min,max]
    max_iter = 100
    step_size = 1
    connect_circle_dist = 20.0
    robot_radius = 0.5        # robot_radius: robot body modeled as circle with given radius
    # Create an RRT* with Reeds-Shepp planner
    planner = RRTStarReedsShepp(start, goal, obs_list, rand_area, max_iter, step_size, connect_circle_dist, robot_radius)

    # Perform RRT* with Reeds-Shepp path planning
    path = planner.planning(animation=True)

    if path is None:
        print("No path found")
        return

    # Plot the path
    plt.figure()

    # Extract and print the path
    print("Path found:")
    for path_node in path:
        print(f"(x: {path_node[0]},\t y: {path_node[1]},\t yaw: {path_node[2]})")
        plt.plot(path_node[0], path_node[1], 'r')

    
    plt.grid(True)
    plt.axis("equal")
    plt.show()

if __name__ == '__main__':
    main()
