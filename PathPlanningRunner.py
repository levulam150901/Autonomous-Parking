#!/usr/bin/env python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
import math

# Import Motion Planning libraries
from PythonRobotics.PathPlanning.RRTStarReedsShepp.rrt_star_reeds_shepp import RRTStarReedsShepp 
from PythonRobotics.PathPlanning.Dijkstra.dijkstra import Dijkstra
from PythonRobotics.PathPlanning.AStar.a_star import AStarPlanner
from PythonRobotics.PathPlanning.DStar.dstar import Dstar, Map, State
from PythonRobotics.PathPlanning.DStarLite.d_star_lite import DStarLite, Node
from PythonRobotics.PathTracking.stanley_controller.stanley_controller import *
from PythonRobotics.PathPlanning.ReedsSheppPath import reeds_shepp_path_planning

def path_planning_RRTStarReedsShepp(start_m, goal_m, obstacle_list_m, show_animation = False):
    # Define search area
    min_x = min(start_m[0], goal_m[0])
    max_x = max(start_m[0], goal_m[0])
    min_y = min(start_m[1], goal_m[1])
    max_y = max(start_m[1], goal_m[1])
    threshold_m = 0.5
    rand_area_x_m = [min_x - threshold_m, max_x + threshold_m]
    rand_area_y_m = [min_y - threshold_m, max_y + threshold_m]

    # Define path planning parameters
    rand_area_m = [rand_area_x_m[0], rand_area_x_m[1], rand_area_y_m[0], rand_area_y_m[1]]
    max_iter = 50
    step_size_m = 0.01
    max_steer_angle_deg = 30
    wheelbase_m = 0.11
    radius_m = wheelbase_m / np.tan(np.deg2rad(max_steer_angle_deg))
    curvature_m = 1.0 / radius_m
    robot_radius_m = 0.2                                            # robot_radius: robot body modeled as circle with given radius
    robot_width_m = 0.13                                            # robot_width: robot body modeled as rectangle with given width
    robot_length_m = 0.17                                           # robot_length: robot body modeled as rectangle with given length
    connect_circle_dist_m = 1.0                                     # rrt search radius to connect nodes

    # Border obstacles in cm
    rand_area_x_cm = [int(rand_area_x_m[0]*100), int(rand_area_x_m[1]*100)]
    rand_area_y_cm = [int(rand_area_y_m[0]*100), int(rand_area_y_m[1]*100)]

    # Create bounding rectangle obstacles around area
    step_size_cm = 10
    x_range_cm = range(rand_area_x_cm[0], rand_area_x_cm[1]+1, step_size_cm)
    y_range_cm = range(rand_area_y_cm[0], rand_area_y_cm[1]+1, step_size_cm)
    for x in x_range_cm:
        for y in y_range_cm:
            if(x == x_range_cm[0]):
                obstacle_list_m.append((x/100.0, y/100.0, step_size_cm/100.0, step_size_cm/100.0, 0.0))       # [x_position_m, y_position_m, radius_m]
            elif(x == x_range_cm[-1]):
                obstacle_list_m.append((x/100.0, y/100.0, step_size_cm/100.0, step_size_cm/100.0, 0.0))       # [x_position_m, y_position_m, radius_m]
            elif(y == y_range_cm[0]):
                obstacle_list_m.append((x/100.0, y/100.0, step_size_cm/100.0, step_size_cm/100.0, 0.0))       # [x_position_m, y_position_m, radius_m]
            elif(y == y_range_cm[-1]):
                obstacle_list_m.append((x/100.0, y/100.0, step_size_cm/100.0, step_size_cm/100.0, 0.0))       # [x_position_m, y_position_m, radius_m]

    for idx, obs  in enumerate(obstacle_list_m):
        obstacle_list_m[idx] = [obs[0], obs[1], obs[2], obs[3], obs[4]]

    # Create an RRT* with Reeds-Shepp planner
    planner = RRTStarReedsShepp(start_m, goal_m, obstacle_list_m, rand_area_m, max_iter, step_size_m, connect_circle_dist_m, curvature_m, robot_width_m, robot_length_m)

    # Perform RRT* with Reeds-Shepp path planning 
    # Plan again and again until it reaches stroke_count lower or equal max_number_of_strokes

    path_length = 0.0
    max_stroke_count = 3
    path = None
    while True:
        # Path planning
        result = planner.planning(animation=show_animation)
        # If no path was found, plan again
        if result is None:
            plt.clf()   # Clear figure
            continue

        path = result[0]
        path_length = result[1]

        # Reverse path to get path from START --> GOAL
        path = path[::-1]
        
        # Count number of strokes by comparing consecutive nodes in path
        path_strokes = 0
        prev_gear = None    # Vehicle gear: True => Forward; False => Backward
        for idx, curr_node in enumerate(path):
            if idx > 1:
                prev_node = path[idx-1]

                # Check position of prev_node and curr_node to define vehicle gear
                curr_gear = point_relative_position(prev_node, curr_node, prev_gear)

                # Count number of strokes (vehicle gear changes) 
                if curr_gear != prev_gear and idx > 2:
                    path_strokes += 1  # Increment the stroke count

                # Save vehicle gear history
                prev_gear = curr_gear
        print(f'PATH STROKES: {path_strokes}')

        # Found a path which fulfills stroke condition
        if path_strokes <= max_stroke_count:
            print(f'PATH FOUND: \n{path}')
            break

    if path and path_length:
        path_length = format(path_length)
        print(f"RRTStarReedsShepp - Total length:", path_length)

    # Draw final path
    if path:  # pragma: no cover
        # RRTStarReedsShepp.draw_graph(planner)
        plt.plot([x for (x, y, yaw) in path], [y for (x, y, yaw) in path], '-r')
        plt.grid(True)
        plt.pause(0.1)
        plt.show()

    # Remove consecutive dublicates in path
    path = [path[i] for i in range(len(path)) if i == 0 or path[i] != path[i - 1]]
        
    # Create path lists [start ... goal]
    rx = []
    ry = []
    ryaw = []
    for x, y, yaw in path:
        rx.append(x)
        ry.append(y)
        ryaw.append(yaw)
    
    # Return path lists [start ... goal] in meters
    return rx, ry, ryaw, path_length

def path_planning_Dijkstra(start_m, goal_m, obstacle_list_m, rand_area_x_m, rand_area_y_m, show_animation=False):
    # Start/Goal in cm
    start_cm = [start_m[0]*100.0, start_m[1]*100.0]
    goal_cm = [goal_m[0]*100.0, goal_m[1]*100.0]

    # Obstacles in cm
    ox_cm = []
    oy_cm = []
    or_cm = []
    for obstacle in obstacle_list_m:
        ox_cm.append(obstacle[0] * 100.0)
        oy_cm.append(obstacle[1] * 100.0)
        or_cm.append(obstacle[2] * 100.0)

    # Border obstacles in cm
    rand_area_x_cm = [int(rand_area_x_m[0]*100), int(rand_area_x_m[1]*100)]
    rand_area_y_cm = [int(rand_area_y_m[0]*100), int(rand_area_y_m[1]*100)]

    step_size_cm = 5
    for x in range(rand_area_x_cm[0], rand_area_x_cm[1]+1):
        for y in range(rand_area_y_cm[0], rand_area_y_cm[1]+1):
            if(x == rand_area_x_cm[0]):
                ox_cm.append(x)
                oy_cm.append(y)
            elif(x == rand_area_x_cm[1]):
                ox_cm.append(x)
                oy_cm.append(y)
            elif(y == rand_area_y_cm[0]):
                ox_cm.append(x)
                oy_cm.append(y)
            elif(y == rand_area_y_cm[1]):
                ox_cm.append(x)
                oy_cm.append(y)

    if show_animation:  # pragma: no cover
        plt.plot(ox_cm, oy_cm, ".k")
        plt.plot(start_cm[0], start_cm[1], "og")
        plt.plot(goal_cm[0], goal_cm[1], "xb")
        plt.grid(True)
        plt.axis("equal")
        # plt.show()
    
    # Compute collision check distance
    safety_r_cm = 5.0      # [cm]
    robot_r_cm = 10.0      # [cm]
    obs_r_cm = max(or_cm)   # [cm]
    collision_check_r = safety_r_cm + robot_r_cm + obs_r_cm
    # Map border obstacles have the same radius as the cuboids --> Issue: currently no path near borders

    # TODO: Use oriented rectangle collision checking instead of circle
    # ...

    planner = Dijkstra(ox_cm,oy_cm,resolution=step_size_cm,robot_radius=10)
    rx, ry, length = planner.planning(start_cm[0], start_cm[1], goal_cm[0], goal_cm[1])

    path_length = format(length)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.01)
        plt.show() 
    
    # Return path lists [start ... goal] in meters
    rx.reverse()
    ry.reverse()
    rx_m = np.array(rx) / 100.0
    ry_m = np.array(ry) / 100.0
    return rx_m, ry_m, path_length

def path_planning_AStarPlanner(start_m, goal_m, obstacle_list_m, rand_area_x_m, rand_area_y_m, show_animation=False):
    # Start/Goal in cm
    start_cm = [start_m[0]*100.0, start_m[1]*100.0]
    goal_cm = [goal_m[0]*100.0, goal_m[1]*100.0]

    # Obstacles in cm
    ox_cm = []
    oy_cm = []
    or_cm = []
    for obstacle in obstacle_list_m:
        ox_cm.append(obstacle[0] * 100.0)
        oy_cm.append(obstacle[1] * 100.0)
        or_cm.append(obstacle[2] * 100.0)

    # Border obstacles in cm
    rand_area_x_cm = [int(rand_area_x_m[0]*100), int(rand_area_x_m[1]*100)]
    rand_area_y_cm = [int(rand_area_y_m[0]*100), int(rand_area_y_m[1]*100)]

    
    for x in range(rand_area_x_cm[0], rand_area_x_cm[1]+1):
        for y in range(rand_area_y_cm[0], rand_area_y_cm[1]+1):
            if(x == rand_area_x_cm[0]):
                ox_cm.append(x)
                oy_cm.append(y)
            elif(x == rand_area_x_cm[1]):
                ox_cm.append(x)
                oy_cm.append(y)
            elif(y == rand_area_y_cm[0]):
                ox_cm.append(x)
                oy_cm.append(y)
            elif(y == rand_area_y_cm[1]):
                ox_cm.append(x)
                oy_cm.append(y)

    if show_animation:  # pragma: no cover
        plt.plot(ox_cm, oy_cm, ".k")
        plt.plot(start_cm[0], start_cm[1], "og")
        plt.plot(goal_cm[0], goal_cm[1], "xb")
        plt.grid(True)
        plt.axis("equal")
        plt.show()

    robot_radius_cm = max(or_cm) + 10     # Use biggest obstacle radius for robot radius
    step_size_cm = 10.0
    a_star = AStarPlanner(ox_cm,oy_cm,resolution=step_size_cm, rr=robot_radius_cm)
    rx, ry, length = a_star.planning(start_cm[0], start_cm[1], goal_cm[0], goal_cm[1])

    path_length = format(length)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()

    # Return path lists [start ... goal] in meters
    rx.reverse()
    ry.reverse()
    rx_m = np.array(rx) / 100.0
    ry_m = np.array(ry) / 100.0
    return rx_m, ry_m, path_length

def path_planning_DStarLite(start_m, goal_m, obstacle_list_m, rand_area_x_m, rand_area_y_m, show_animation=False):
    # Start/Goal in cm
    start_cm = [int(start_m[0]*100.0), int(start_m[1]*100.0)]
    goal_cm = [int(goal_m[0]*100.0), int(goal_m[1]*100.0)]

    # Obstacles in cm
    ox_cm = []
    oy_cm = [] 
    for obstacle in obstacle_list_m:
        ox_cm.append(int(obstacle[0] * 100.0))
        oy_cm.append(int(obstacle[1] * 100.0))

    # Border obstacles in cm
    rand_area_x_cm = [int(rand_area_x_m[0]*100), int(rand_area_x_m[1]*100)]
    rand_area_y_cm = [int(rand_area_y_m[0]*100), int(rand_area_y_m[1]*100)]

    step_size_cm = 10
    for x in range(rand_area_x_cm[0], rand_area_x_cm[1]+1, step_size_cm):
        for y in range(rand_area_y_cm[0], rand_area_y_cm[1]+1, step_size_cm):
            if(x == rand_area_x_cm[0]):
                ox_cm.append(x)
                oy_cm.append(y)
            elif(x == rand_area_x_cm[1]):
                ox_cm.append(x)
                oy_cm.append(y)
            elif(y == rand_area_y_cm[0]):
                ox_cm.append(x)
                oy_cm.append(y)
            elif(y == rand_area_y_cm[1]):
                ox_cm.append(x)
                oy_cm.append(y)
    
    if show_animation:
        plt.plot(ox_cm, oy_cm, ".k")
        plt.plot(start_cm[0], start_cm[1], "og")
        plt.plot(goal_cm[0], goal_cm[1], "xb")
        plt.grid(True)
        plt.axis("equal")
        label_column = ['Start', 'Goal', 'Path taken',
                        'Current computed path', 'Previous computed path',
                        'Obstacles']
        columns = [plt.plot([], [], symbol, color=colour, alpha=alpha)[0]
                   for symbol, colour, alpha in [['o', 'g', 1],
                                                 ['x', 'b', 1],
                                                 ['-', 'r', 1],
                                                 ['.', 'c', 1],
                                                 ['.', 'c', 0.3],
                                                 ['.', 'k', 1]]]
        plt.legend(columns, label_column, bbox_to_anchor=(1, 1), title="Key:",
                   fontsize="xx-small")
        plt.plot()
        plt.pause(1)

    # Obstacles discovered at time = row
    # time = 1, obstacles discovered at (0, 2), (9, 2), (4, 0)
    # time = 2, obstacles discovered at (0, 1), (7, 7)
    # ...
    # when the spoofed obstacles are:
    # spoofed_ox = [[0, 9, 4], [0, 7], [], [], [], [], [], [5]]
    # spoofed_oy = [[2, 2, 0], [1, 7], [], [], [], [], [], [4]]

    # Reroute
    # spoofed_ox = [[], [], [], [], [], [], [], [40 for _ in range(10, 21)]]
    # spoofed_oy = [[], [], [], [], [], [], [], [i for i in range(10, 21)]]

    # Obstacles that demostrate large rerouting
    # spoofed_ox = [[], [], [],
    #               [i for i in range(0, 21)] + [0 for _ in range(0, 20)]]
    # spoofed_oy = [[], [], [],
    #               [20 for _ in range(0, 21)] + [i for i in range(0, 20)]]


    dstarlite = DStarLite(ox_cm, oy_cm)
    # dstarlite.main(Node(x=start_cm[0], y=start_cm[1]), Node(x=goal_cm[0], y=goal_cm[1]),
    #                spoofed_ox=spoofed_ox, spoofed_oy=spoofed_oy)
    rx, ry = dstarlite.main(Node(x=start_cm[0], y=start_cm[1]), Node(x=goal_cm[0], y=goal_cm[1]), spoofed_ox=[], spoofed_oy=[])
    
    # Return path lists [start ... goal] in meters
    rx_m = np.array(rx) / 100.0
    ry_m = np.array(ry) / 100.0
    return rx_m, ry_m

def path_planning_DStar(start_m, goal_m, obstacle_list_m, rand_area_x_m, rand_area_y_m, show_animation=False):
    # Start/Goal in cm
    start_cm = [int(start_m[0]*100.0), int(start_m[1]*100.0)]
    goal_cm = [int(goal_m[0]*100.0), int(goal_m[1]*100.0)]

    # Obstacles in cm
    ox_cm = []
    oy_cm = [] 
    for obstacle in obstacle_list_m:
        ox_cm.append(int(obstacle[0] * 100.0))
        oy_cm.append(int(obstacle[1] * 100.0))

    # Border obstacles in cm
    rand_area_x_cm = [int(rand_area_x_m[0]*100), int(rand_area_x_m[1]*100)]
    rand_area_y_cm = [int(rand_area_y_m[0]*100), int(rand_area_y_m[1]*100)]

    step_size_cm = 10
    for x in range(rand_area_x_cm[0], rand_area_x_cm[1]+1, step_size_cm):
        for y in range(rand_area_y_cm[0], rand_area_y_cm[1]+1, step_size_cm):
            if(x == rand_area_x_cm[0]):
                ox_cm.append(x)
                oy_cm.append(y)
            elif(x == rand_area_x_cm[1]):
                ox_cm.append(x)
                oy_cm.append(y)
            elif(y == rand_area_y_cm[0]):
                ox_cm.append(x)
                oy_cm.append(y)
            elif(y == rand_area_y_cm[1]):
                ox_cm.append(x)
                oy_cm.append(y)


    m = Map(ox_cm, oy_cm, start_cm, goal_cm)

    # print([(i, j) for i, j in zip(ox_cm, oy_cm)])
    m.set_obstacle([(i, j) for i, j in zip(ox_cm, oy_cm)])

    if show_animation:
        plt.plot(ox_cm, oy_cm, ".k")
        plt.plot(start_cm[0], start_cm[1], "og")
        plt.plot(goal_cm[0], goal_cm[1], "xb")
        plt.axis("equal")

    dstar = Dstar(m)
    rx, ry, length = dstar.run(m.start, m.end)

    path_length = format(length)

    if show_animation:
        rx = m.get_x_position_list_without_offset(rx)
        ry = m.get_y_position_list_without_offset(ry)
        plt.plot(rx, ry, "-r")
        plt.show()

    # Return path lists [start ... goal] in meters
    rx_m = np.array(rx) / 100.0
    ry_m = np.array(ry) / 100.0
    return rx_m, ry_m, path_length

def path_tracking_StanleyController(start_m, ax_m, ay_m, target_speed_m_s, show_animation=False):
    """Plot an example of Stanley steering control on a cubic spline."""
    # Start/Goal in cm
    start_cm = [int(start_m[0]*100.0), int(start_m[1]*100.0), start_m[2]]   # [x_cm, y_cm, yaw_rad] 

    # Path in cm
    ax = [int(x * 100) for x in ax_m]
    ay = [int(y * 100) for y in ay_m]
    # Compute spline based on input path
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=30)  # ds = distance sampling = distance between point on spline

    target_speed_cm_s = target_speed_m_s * 100  # [cm/s]

    max_simulation_time = 100.0

    # Initial state
    state = State(x=start_cm[0], y=start_cm[1], yaw=start_cm[2], v=0.0)

    last_idx = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_idx, _ = calc_target_index(state, cx, cy)

    while max_simulation_time >= time and last_idx > target_idx:
        ai = pid_control(target_speed_cm_s, state.v)
        di, target_idx = stanley_control(state, cx, cy, cyaw, abc)
        state.update(ai, di)

        time += dt

        # Output format:
        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if show_animation:  # pragma: no cover
            # plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[cm/s]:" + str(state.v)[:4])
            plt.pause(0.001)

    # Test
    assert last_idx >= target_idx, "Cannot reach goal"

    if show_animation:  # pragma: no cover
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(t, [iv * 3.6 for iv in v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()

    # Return lists of x, y, yaw, v, t
    return x, y, yaw, v, t  


def point_relative_position(prev_point, next_point, prev_gear):
    # Extract information from previous and next points
    prev_x, prev_y, prev_yaw = prev_point[0], prev_point[1], prev_point[2]
    next_x, next_y = next_point[0], next_point[1]

    # Calculate displacement vector from previous point to next point
    displacement_x = next_x - prev_x
    displacement_y = next_y - prev_y

    # Calculate the direction vector based on the previous point's yaw angle
    direction_x = math.cos(prev_yaw)
    direction_y = math.sin(prev_yaw)

    # Calculate the dot product of direction vector and displacement vector
    dot_product = direction_x * displacement_x + direction_y * displacement_y

    # Determine the relative position based on the sign of the dot product
    if dot_product > 0:
        # Next point is infront of previous point --> Forward gear
        return True
    elif dot_product < 0:
        #  Next point is behind the previous point --> Backward gear
        return False
    else:
        # Next point is same on the previous point --> Same gear as before 
        return prev_gear


if __name__ == '__main__':
    # Define the start and goal positions
    start = (-1.85, 0.1325, -np.pi/2)                  # Start Position [x,y, start dir in rad]
    goal = (-1.00, -0.50, 0.0)                     # Goal Position [x,y, goal dir in rad]

    # Define search area
    # rand_area_x_m = [-2.5, 0]
    # rand_area_y_m = [-2.5, 0]
    rand_area_x_m = [-2.5, 0]
    rand_area_y_m = [0, 2.5]

    # Define obstacle list [x position_m, y position_m, radius_m]
    obstacle_list = [[-1.3749988998291784, 1.375000128910268, 0.25, 0.25, 1.9735661080454134e-08], [-1.3750000884965963, 1.625000432029795, 0.25, 0.25, 8.211307707899254e-08], [-1.375000230425759, 1.875000805427043, 0.25, 0.25, -2.4020823895257e-07], [-1.3750039862386922, 2.125002489301276, 0.25, 0.25, -5.904768261757853e-08], [-1.375007027990223, 1.1249963851911506, 0.25, 0.25, 1.1339679431489447e-05], [-1.1250034221984053, 1.1250076953832377, 0.25, 0.25, 4.647045731910574e-05], [-0.8749892472055414, 1.125015403398453, 0.25, 0.25, 5.903712248616574e-06], [-0.8750000670440002, 0.8750106288420382, 0.25, 0.25, -1.4275306204685798e-05], [-0.8750022838851056, 0.6250103154977967, 0.25, 0.25, -1.4463977517448188e-05], [-0.8749885064872707, 0.3750001428132739, 0.25, 0.25, -9.411429039573819e-05], [-0.8749978362024249, 0.12499923812843783, 0.25, 0.25, -9.486154460594638e-05], [-0.8749987578926902, -0.12501242744493973, 0.25, 0.25, 4.813394382044834e-08], [-1.1250026391592498, -0.12500134394135037, 0.25, 0.25, -1.5486476069577747e-06], [-1.3750037733287015, -0.12499925336079447, 0.25, 0.25, -1.2155567678527916e-07], [-0.8749998576763511, -0.8749992918358481, 0.25, 0.25, 3.241037769204721e-06], [-1.1250008202302646, -0.8749984416690622, 0.25, 0.25, 1.4828153322353408e-06], [-1.3750013672761283, -0.8750003131859605, 0.25, 0.25, -2.696376686353382e-07], [-0.624991282832163, -0.8750109533594129, 0.25, 0.25, -9.108294642649644e-07], [-0.6249961585746708, -0.6250102520458617, 0.25, 0.25, -4.444369647584517e-07], [-0.6249865013939683, -0.37500988759547166, 0.25, 0.25, -9.249245979583684e-07], [-0.6249745852346407, -0.12500952625699208, 0.25, 0.25, 1.1112701874685841e-06], [-0.8749996666010157, -1.124999996317431, 0.25, 0.25, -5.656139334819719e-07], [-0.8750022743013857, -1.3750002197360136, 0.25, 0.25, -7.570795001227611e-08], [-0.8750026228707772, -1.625000293946456, 0.25, 0.25, -6.934637988706109e-08], [-0.875000683856389, -1.8750005431133077, 0.25, 0.25, 1.194317538417912e-06], [-0.8750003374682109, -2.125000809975315, 0.25, 0.25, 7.52700819988477e-08], [-1.124990544183853, 2.3500004688132425, 0.25, 0.25, 1.6852396775407579e-07], [-0.6250000300796363, -2.32500130625383, 0.25, 0.25, -6.128405607827895e-08], [-1.1249953168168734, 2.0999996600800737, 0.25, 0.25, -2.0100946659164694e-08], [-0.6249995885987827, -2.074999923989746, 0.25, 0.25, 1.0713491147601574e-07], [-2.35, -1.0, 0.1, 0.1, 0.0], [-2.35, -0.9, 0.1, 0.1, 0.0], [-2.35, -0.8, 0.1, 0.1, 0.0], [-2.35, -0.7, 0.1, 0.1, 0.0], [-2.35, -0.6, 0.1, 0.1, 0.0], [-2.35, -0.5, 0.1, 0.1, 0.0], [-2.35, -0.4, 0.1, 0.1, 0.0], [-2.35, -0.3, 0.1, 0.1, 0.0], [-2.35, -0.2, 0.1, 0.1, 0.0], [-2.35, -0.1, 0.1, 0.1, 0.0], [-2.35, 0.0, 0.1, 0.1, 0.0], [-2.35, 0.1, 0.1, 0.1, 0.0], [-2.35, 0.2, 0.1, 0.1, 0.0], [-2.35, 0.3, 0.1, 0.1, 0.0], [-2.35, 0.4, 0.1, 0.1, 0.0], [-2.35, 0.5, 0.1, 0.1, 0.0], [-2.35, 0.6, 0.1, 0.1, 0.0], [-2.25, -1.0, 0.1, 0.1, 0.0], [-2.25, 0.6, 0.1, 0.1, 0.0], [-2.15, -1.0, 0.1, 0.1, 0.0], [-2.15, 0.6, 0.1, 0.1, 0.0], [-2.05, -1.0, 0.1, 0.1, 0.0], [-2.05, 0.6, 0.1, 0.1, 0.0], [-1.95, -1.0, 0.1, 0.1, 0.0], [-1.95, 0.6, 0.1, 0.1, 0.0], [-1.85, -1.0, 0.1, 0.1, 0.0], [-1.85, 0.6, 0.1, 0.1, 0.0], [-1.75, -1.0, 0.1, 0.1, 0.0], [-1.75, 0.6, 0.1, 0.1, 0.0], [-1.65, -1.0, 0.1, 0.1, 0.0], [-1.65, 0.6, 0.1, 0.1, 0.0], [-1.55, -1.0, 0.1, 0.1, 0.0], [-1.55, 0.6, 0.1, 0.1, 0.0], [-1.45, -1.0, 0.1, 0.1, 0.0], [-1.45, 0.6, 0.1, 0.1, 0.0], [-1.35, -1.0, 0.1, 0.1, 0.0], [-1.35, 0.6, 0.1, 0.1, 0.0], [-1.25, -1.0, 0.1, 0.1, 0.0], [-1.25, 0.6, 0.1, 0.1, 0.0], [-1.15, -1.0, 0.1, 0.1, 0.0], [-1.15, 0.6, 0.1, 0.1, 0.0], [-1.05, -1.0, 0.1, 0.1, 0.0], [-1.05, 0.6, 0.1, 0.1, 0.0], [-0.95, -1.0, 0.1, 0.1, 0.0], [-0.95, 0.6, 0.1, 0.1, 0.0], [-0.85, -1.0, 0.1, 0.1, 0.0], [-0.85, 0.6, 0.1, 0.1, 0.0], [-0.75, -1.0, 0.1, 0.1, 0.0], [-0.75, 0.6, 0.1, 0.1, 0.0], [-0.65, -1.0, 0.1, 0.1, 0.0], [-0.65, 0.6, 0.1, 0.1, 0.0], [-0.55, -1.0, 0.1, 0.1, 0.0], [-0.55, -0.9, 0.1, 0.1, 0.0], [-0.55, -0.8, 0.1, 0.1, 0.0], [-0.55, -0.7, 0.1, 0.1, 0.0], [-0.55, -0.6, 0.1, 0.1, 0.0], [-0.55, -0.5, 0.1, 0.1, 0.0], [-0.55, -0.4, 0.1, 0.1, 0.0], [-0.55, -0.3, 0.1, 0.1, 0.0], [-0.55, -0.2, 0.1, 0.1, 0.0], [-0.55, -0.1, 0.1, 0.1, 0.0], [-0.55, 0.0, 0.1, 0.1, 0.0], [-0.55, 0.1, 0.1, 0.1, 0.0], [-0.55, 0.2, 0.1, 0.1, 0.0], [-0.55, 0.3, 0.1, 0.1, 0.0], [-0.55, 0.4, 0.1, 0.1, 0.0], [-0.55, 0.5, 0.1, 0.1, 0.0], [-0.55, 0.6, 0.1, 0.1, 0.0]]

    # Path Planning
    rx_m, ry_m, ryaw, length = path_planning_RRTStarReedsShepp(start, goal, obstacle_list, show_animation=True)

    # Path Tracking
    # x_m, y_m, yaw, v_m_s, t_s  = path_tracking_StanleyController(start, rx_m, ry_m, target_speed_m_s=0.5, show_animation=True)

    print("")