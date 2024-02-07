#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'ar1'

import time
import sys
from PythonRobotics.PathPlanning.RRTStarReedsShepp.rrt_star_reeds_shepp import RRTStarReedsShepp 


import numpy as np
import matplotlib.pyplot as plt
import cv2
from PathPlanningRunner import *

from coppeliasim_zmqremoteapi_client import *
from libs import *
class Obstacle:
    def __init__(self, sim, obstacle_name):
        self.obstacle_handle = sim.getObjectHandle(obstacle_name)

        # Get the object's bounding box parameters in local coordinates
        local_min_x = sim.getObjectFloatParam(self.obstacle_handle, sim.objfloatparam_objbbox_min_x)
        local_max_x = sim.getObjectFloatParam(self.obstacle_handle, sim.objfloatparam_objbbox_max_x)

        local_min_y = sim.getObjectFloatParam(self.obstacle_handle, sim.objfloatparam_objbbox_min_y)
        local_max_y = sim.getObjectFloatParam(self.obstacle_handle, sim.objfloatparam_objbbox_max_y)

        local_min_z = sim.getObjectFloatParam(self.obstacle_handle, sim.objfloatparam_objbbox_min_z)
        local_max_z = sim.getObjectFloatParam(self.obstacle_handle, sim.objfloatparam_objbbox_max_z)

        # Get the object's position and orientation in world coordinates
        self.world_position = sim.getObjectPosition(self.obstacle_handle, sim.handle_world)
        self.world_orientation = sim.getObjectOrientation(self.obstacle_handle, sim.handle_world)

        # Transform the local bounding box dimensions to world coordinates
        self.world_min_x = self.world_position[0] + local_min_x * (1 if self.world_orientation[0] <= 0 else -1)
        self.world_max_x = self.world_position[0] + local_max_x * (1 if self.world_orientation[0] <= 0 else -1)

        self.world_min_y = self.world_position[1] + local_min_y * (1 if self.world_orientation[1] <= 0 else -1)
        self.world_max_y = self.world_position[1] + local_max_y * (1 if self.world_orientation[1] <= 0 else -1)

        self.world_min_z = self.world_position[2] + local_min_z * (1 if self.world_orientation[2] <= 0 else -1)
        self.world_max_z = self.world_position[2] + local_max_z * (1 if self.world_orientation[2] <= 0 else -1)

        # Calculate the dimensions of the bounding box in world coordinates
        # bbox_dimensions = [
        #     world_max_x - world_min_x,
        #     world_max_y - world_min_y,
        #     world_max_z - world_min_z
        # ]

        # print("Bounding Box Dimensions (X, Y, Z) in World Coordinates:", bbox_dimensions)

        # Get the object's vertices
        # ...

#####################################################################################################################
class DyorBase:
    def __init__(self, target_name='target', bot_name='Bot', wheel_speed=1.0):

        self.TARGET_NAME = target_name
        self.BOT_NAME = bot_name

        # Constructive parameters
        self.wheel_radius = 0.03 # wheel radius
        self.b = 0.0565 # wheel base (wheel separation distance)
        self.sepUS = 0.0815 # distance of ultrasonic sensor (US) with respect to the robot wheels

        self.a = 0.03 # Separation distance between front and rear LIDAR sensors

        self.sepLIDAR = 0.041 # Lateral separation distance of LIDAR sensors w.r.t.the robot
        self.vref = 0.05* wheel_speed # velocity when following the wall or moving forward
        self.vTurn = 0.005 # velocity when turning
        self.objectDist = 0.3 # distance to stop the robot when detecting an object
        self.min_distUS = self.objectDist - self.sepUS # Distance to stop when there's a wall in front of the robot

        self.min_distUS = self.objectDist - self.sepUS # Distance to stop when there's a wall in front of the robot
        self.kWall = 0.0 # Wall following gain
        self.e = 0.1 # Distance of an off-center point to perform the kinematic control for wall following
        self.wallDist = self.objectDist - self.sepLIDAR # The expected distance to detect a wall with LIDAR sensors
        self.goalDetectedTol = 0.1 # Tolerance to point to the goal
        self.wallDetectedTol = 0 # Tolerance to determine that there's a wall when rotating
        self.goalReachedTol = 0 # Tolerance to determine the robot has reached the goal
        self.positionTol = 0.01

        self.direction = -1 # Follow right 1, follow left - 1

        self.max_distUS = 0.5 # Maximum distance returned by the US sensor
        self.dUS = self.max_distUS # Initial US distance
        self.max_distLIDAR = 1.2 # Maximum distance returned by the LIDAR sensors
        self.d_FR = self.max_distLIDAR # Initial LIDAR distance
        self.d_RR = self.max_distLIDAR # Initial LIDAR distance

        self.d_FL = self.max_distLIDAR # Initial LIDAR distance
        self.d_RL = self.max_distLIDAR # Initial LIDAR distance

        self.SLEEP_TIME = 0.2
        self.PI = math.pi
        self.rad_to_deg = 180.0 / self.PI
        self.deg_to_rad = 1.0 / self.rad_to_deg

        self.bot_dir = None
        self.bot_pos = None
        self.bot_euler_angles = None
        self.bot_left_vel_m_s = 0.0
        self.bot_right_vel_m_s = 0.0

        self.target_pos = None
        self.target_dir = None
        self.initialized_stanley = False
        self.idx = 0

        self.sensor_with_minimal_distance = None  # the proximity sensor with the minimal distance

        self.track_length = 0.0
        self.prev_pos = None

        # handles for the sensors
        self.us = None
        self.ir_FL = None
        self.ir_FR = None
        self.ir_RL = None
        self.ir_RR = None

        self.us_distance = None
        #self.d_FL = None
        #self.d_FR = None
        #self.d_RL = None
        #self.d_RR = None

        self.last_hitpoint = None
        self.last_leavepoint = None
        self.left_dir = None
        self.us_dist_thresh = 0.25                              # Obstacle detection distance for US sensor
        self.lidar_dist_thresh = self.us_dist_thresh - 0.05     # Obstacle rounding distance for LIDAR sensor

        self.obstacle_list = []

        self._init_client_id()
        self._init_handles()
        # self._init_sensor_handles()

        self.about = ''
        self.print_about_info()
#######################################################################################
    def _init_client_id(self):
        # connect to Remote API
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')

#######################################################################################
    def _init_handles(self):
        self._init_wheels_handle()
        self._init_target_handle()
        self._init_robot_handle()
        # self._init_obstacle_handle()


########################################################################################################################
    def _init_obstacle_handle(self):
        # Fill obstacle list with obstacles from scene
        obstacle_name = "Cuboid"
        max_number_of_obstacles = 200

        for index in range(max_number_of_obstacles):
            # Receive obstacle handle
            path = f'/{obstacle_name}'
            options = {"index": index, "noError" : True}
            # print(f'Get obstacle position and shape {path}[{index}]')
            objectHandle = self.sim.getObject(path, options)

            if(objectHandle == -1):
                # Stop when an obstacle isn't found
                break
            else:
                # Receive obstacle position and shape information
                obs_world_position = self.sim.getObjectPosition(objectHandle, self.sim.handle_world)
                obs_world_euler_orientation = self.sim.getObjectOrientation(objectHandle, self.sim.handle_world)
                result, pureType, dimensions = self.sim.getShapeGeomInfo(objectHandle)
                obs_width_m = self.sim.getObjectFloatParameter(objectHandle, self.sim.objfloatparam_objbbox_max_x)
                obs_length_m = self.sim.getObjectFloatParameter(objectHandle, self.sim.objfloatparam_objbbox_max_y)


                # Circular obstacle
                x_m = obs_world_position[0]
                y_m = obs_world_position[1]
                radius_m = math.sqrt((dimensions[0]/2)**2 + (dimensions[1]/2)**2)

                # Add obstacle to list [x,y,size]
                # self.obstacle_list.append([x_m, y_m, radius_m])
                self.obstacle_list.append([x_m, y_m, obs_width_m[1] * 2 , obs_length_m[1] * 2, obs_world_euler_orientation[2]])

                # print(f'[{x_m}, {y_m}, {radius_m}],')
                

                # Oriented rectangular obstacle [(x0_m, y0_m), (x1_m, y1_m), (x2_m, y2_m), (x3_m, y3_m)]
                # x_m = obs_world_position[0]
                # y_m = obs_world_position[1]
                # euler_yaw_rad = obs_world_euler_orientation[2]  # Yaw
                # # TODO
                # rectangle_coords = [(0, 0), (2, 0), (2, 3), (0, 3)]
                # Add obstacle to list [x,y,size]
                # self.obstacle_list.append([x_m, y_m, radius_m])


########################################################################################################################
    def _init_robot_handle(self):
        # handle of robot
        self.bot_handle = self.sim.getObjectHandle(self.BOT_NAME)

########################################################################################################################
    def _init_target_handle(self):
        # get handle of target robot
        self.target_handle = self.sim.getObjectHandle(self.TARGET_NAME)

########################################################################################################################
    def _init_wheels_handle(self):
        # get handles of robot wheels
            self.left_front_motor_handle = self.sim.getObjectHandle('left_front_joint')
            self.left_rear_motor_handle = self.sim.getObjectHandle('left_rear_joint')
            self.right_front_motor_handle = self.sim.getObjectHandle('right_front_joint')
            self.right_rear_motor_handle = self.sim.getObjectHandle('right_rear_joint')

########################################################################################################################
    def _init_sensor_handles(self):
        self.us = self.sim.getObjectHandle('Proximity_sensor')
        self.ir_FL = self.sim.getObjectHandle('ir_front_left')
        self.ir_FR = self.sim.getObjectHandle('ir_front_right')
        self.ir_RL = self.sim.getObjectHandle('ir_rear_left')
        self.ir_RR = self.sim.getObjectHandle('ir_rear_right')

######################################################################################################
    def _init_values(self):
        _ = self.sim.getObjectPosition(self.target_handle, -1)
        _ = self.sim.getObjectPosition(self.bot_handle, -1)
        _ = self.sim.getObjectOrientation(self.bot_handle, -1)

######################################################################################################
    def read_values(self):
        target_pos = self.sim.getObjectPosition(self.target_handle, -1)
        self.target_pos = Vector3(x=target_pos[0], y=target_pos[1], z=target_pos[2])

        bot_pos = self.sim.getObjectPosition(self.bot_handle, -1)
        self.bot_pos = Vector3(x=bot_pos[0], y=bot_pos[1], z=bot_pos[2])

        bot_euler_angles = self.sim.getObjectOrientation(self.bot_handle, -1)
        self.bot_euler_angles = Vector3(x=bot_euler_angles[0], y=bot_euler_angles[1], z=bot_euler_angles[2])

        target_euler_angles = self.sim.getObjectOrientation(self.target_handle, -1)
        self.target_euler_angles = Vector3(x=target_euler_angles[0], y=target_euler_angles[1], z=target_euler_angles[2])

######################################################################################################
    def stop_move(self):
        #print("stop")
            self.sim.setJointTargetVelocity(self.left_front_motor_handle,  0)
            self.sim.setJointTargetVelocity(self.left_rear_motor_handle,  0)
            self.sim.setJointTargetVelocity(self.right_front_motor_handle, 0)
            self.sim.setJointTargetVelocity(self.right_rear_motor_handle, 0)
############################################################################################################
    def read_US_sensor(self):
        detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = self.sim.readProximitySensor(self.us)
        # int res, float dist, list point, int obj, list n
        dist = math.sqrt(detected_point[0] ** 2 + detected_point[1] ** 2 + detected_point[2] ** 2)
        if dist > self.max_distUS or detection_state is False:
            self.us_distance = self.max_distUS
        else:
            self.us_distance = dist

############################################################################################################
    def read_LIDAR_sensors(self):
        # front left
        detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = self.sim.readProximitySensor(self.ir_FL)
        dist = math.sqrt(detected_point[0] ** 2 + detected_point[1] ** 2 + detected_point[2] ** 2)
        if dist > self.max_distLIDAR or detection_state is False:
            self.d_FL = self.max_distLIDAR
        else:
            self.d_FL = dist
        # front right
        detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = self.sim.readProximitySensor(self.ir_FR)
        dist = math.sqrt(detected_point[0] ** 2 + detected_point[1] ** 2 + detected_point[2] ** 2)
        if dist > self.max_distLIDAR or detection_state is False:
            self.d_FR = self.max_distLIDAR
        else:
            self.d_FR = dist
        # rear left
        detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = self.sim.readProximitySensor(self.ir_RL)
        dist = math.sqrt(detected_point[0] ** 2 + detected_point[1] ** 2 + detected_point[2] ** 2)
        if dist > self.max_distLIDAR or detection_state is False:
            self.d_RL = self.max_distLIDAR
        else:
            self.d_RL = dist
        # rear right
        detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = self.sim.readProximitySensor(self.ir_RR)
        dist = math.sqrt(detected_point[0] ** 2 + detected_point[1] ** 2 + detected_point[2] ** 2)
        if dist > self.max_distLIDAR or detection_state is False:
            self.d_RR = self.max_distLIDAR
        else:
            self.d_RR = dist

#####################################################################################################
    def print_about_info(self):
        print("Algorithm: {0}\nTarget name: {1}\nBot name: {2}\nSpeed of wheel: {3}".format(self.about, self.TARGET_NAME, self.BOT_NAME, self.vref))
#####################################################################################################
    def calc_lenght_of_robot_track(self):

        if self.prev_pos is None:
            self.prev_pos = self.bot_pos

        self.track_length += distance_between_points(self.prev_pos, self.bot_pos)
        self.prev_pos = self.bot_pos

        print(self.track_length)
####################################################################################
    def tick(self):
        time.sleep(self.SLEEP_TIME)

    def loop(self):
        pass

    def action_moving(self):
        pass

    def action_rotating(self):
        pass

    def action_rounding(self):
        pass
#####################################################################################################
    def dyor_path_planning(self):

        # Define the start and goal positions
        start_m = (self.bot_pos.x, self.bot_pos.y,  self.bot_euler_angles.z)                # Start Position [x,y, start dir in rad]       
        goal_m = (self.target_pos.x, self.target_pos.y, self.target_euler_angles.z)             # Goal Position [x,y, goal dir in rad]
        print(start_m)
        print(goal_m)
        # Define path planning parameters
        rand_area_x_m = [-2.5, 2.5]
        rand_area_y_m = [-2.5, 2.5]
        
        # Define obstacle list
        obstacle_list_m = self.obstacle_list

        start_time_ns = time.time_ns()
        
        # Path Planning
        # rx_m, ry_m = path_planning_Dijkstra(start_m, goal_m, obstacle_list_m, rand_area_x_m, rand_area_y_m, show_animation=False)
        # rx_m, ry_m = path_planning_AStarPlanner(start_m, goal_m, obstacle_list_m, rand_area_x_m, rand_area_y_m, show_animation=False)
        # rx_m, ry_m = path_planning_DStar(start_m, goal_m, obstacle_list_m, rand_area_x_m, rand_area_y_m, show_animation=True)
        # rx_m, ry_m = path_planning_DStarLite(start_m, goal_m, obstacle_list_m, rand_area_x_m, rand_area_y_m, show_animation=True)
        rx_m, ry_m, ryaw, path_length = path_planning_RRTStarReedsShepp(start_m, goal_m, obstacle_list_m, show_animation=True)

        end_time_ns = time.time_ns()
        execution_time = (end_time_ns - start_time_ns) / 1000000.0

        print(f"Total runtime ms:", execution_time)
        print()

        return rx_m, ry_m, ryaw


#####################################################################################################
    def dyor_path_tracking(self, rx_m, ry_m, ryaw_rad):
        # Define the start and goal positions
        start_m = (self.bot_pos.x, self.bot_pos.y,  self.bot_euler_angles.z)                # Start Position [x,y, start dir in rad]  

        # Path Tracking Algorithm
        yaw_rad, v_m_s  = self.path_tracking_StanleyController_Bug2(start_m, v_start_m_s = 0.0, ax_m = rx_m, ay_m = ry_m, ayaw_rad = ryaw_rad, target_speed_m_s=0.05, show_animation=True)

        return yaw_rad, v_m_s
    
#####################################################################################################
    def path_tracking_StanleyController_Bug2(self, start_m, v_start_m_s, ax_m, ay_m, ayaw_rad, target_speed_m_s, show_animation):
        """Plot an example of Stanley steering control on a cubic spline."""
        # Init
        if self.initialized_stanley is False:
        
            # Start/Goal in cm
            self.start_cm = [int(start_m[0]*100.0), int(start_m[1]*100.0), start_m[2]]   # [x_cm, y_cm, yaw_rad] 

            # Path in cm
            ax = [int(x * 100) for x in ax_m]
            ay = [int(y * 100) for y in ay_m]

            # Compute spline based on input path
            # self.cx, self.cy, self.cyaw, self.ck, self.s = cubic_spline_planner.calc_spline_course(ax, ay, ds=1)  # ds = distance sampling = distance between point on spline
            self.cx = ax; self.cy = ay; self.cyaw = ayaw_rad    # Use ReedShepp path instead of cubic spline
    
            # Compute path_splitting_indices 
            # Format: [(start_idx1, end_idx1), (start_idx2, end_idx2), ... ]
            prev_gear = None    # Vehicle gear: True => Forward; False => Backward
            start_idx = 0
            path = []
            self.path_splitting_indices = []
            path = list(zip(self.cx, self.cy, self.cyaw))
            for idx, curr_node in enumerate(path):
                if idx > 1:
                    prev_node = path[idx-1]

                    # Check position of prev_node and curr_node to define vehicle gear
                    curr_gear = point_relative_position(prev_node, curr_node, prev_gear)

                    # Count number of strokes (vehicle gear changes) 
                    if (curr_gear != prev_gear and idx > 2) or idx == (len(path)-1):
                        end_idx = idx
                        self.path_splitting_indices.append((start_idx, end_idx))
                        start_idx = end_idx

                    # Save vehicle gear history
                    prev_gear = curr_gear

            # Plot spline in Coppeliasim
            path_points = []
            for x, y in zip(self.cx, self.cy):
                # [x y z qx qy qz qw]
                path_points += [x/100.0, y/100.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            path_handle = self.sim.createPath(path_points, 0, 1000, 0)
            
            # Speeds in cm/s
            self.target_speed_cm_s = target_speed_m_s * 100.0  # [cm/s]
            v_start_cm_s = v_start_m_s * 100.0 # [cm/s]

            # Initial state
            L = 11.0/2.0  # [cm] Wheel base of vehicle RVR
            middle_axis_x = self.start_cm[0] + L * np.cos(self.start_cm[2])
            middle_axis_y = self.start_cm[1] + L * np.sin(self.start_cm[2])
            self.state = State(x=middle_axis_x, y=middle_axis_y, yaw=self.start_cm[2], v=v_start_cm_s)
            self.middle_axis_x_list = [middle_axis_x]   # [cm]
            self.middle_axis_y_list = [middle_axis_y]   # [cm]           

            # State history
            self.x = [self.state.x]
            self.y = [self.state.y]
            self.yaw = [self.state.yaw]
            self.v = [self.state.v]
            self.t = [self.sim.getSimulationTime()]
            self.target_idx, _ = calc_target_index(self.state, self.cx, self.cy)
            self.forward = point_relative_position(prev_point=(self.cx[self.target_idx], self.cy[self.target_idx], self.cyaw[self.target_idx]), \
                                                   next_point=(self.cx[self.target_idx + 1], self.cy[self.target_idx + 1], self.cyaw[self.target_idx + 1]), prev_gear=None)

            # Track count, where each track is a part of the whole path having the same direction
            self.track_idx = 0
            
            # Create plot
            plt.figure()

            # Initialization finished
            self.initialized_stanley = True


        # Controlling
        t_sec = self.sim.getSimulationTime()
        dt = t_sec - self.t[-1]                 # [s]
        # dt = 0.1               # [s]
        
        # Overwrite system state (for closed loop)
        self.state.x = self.bot_pos.x * 100.0       # [cm]
        self.state.y = self.bot_pos.y * 100.0       # [cm] 
        self.state.yaw = self.bot_euler_angles.z    # [rad] 
        v_forward_m_s = (self.bot_left_vel_m_s + self.bot_right_vel_m_s) / 2                # Forward
        self.state.v = v_forward_m_s * 100.0        # [cm/s]
        wheelbase_m = 0.16 # distance between left and right wheel
        v_angular_m_s = (self.bot_left_vel_m_s - self.bot_right_vel_m_s) / wheelbase_m        # Around RVR axis
        self.state.omega = v_angular_m_s * 100.0    # [cm/s]

        # Compute center axis for plotting
        L = 11.0/2.0  # [cm] Wheel base of vehicle RVR
        middle_axis_x = self.state.x + L * np.cos(self.state.yaw)
        middle_axis_y = self.state.y + L * np.sin(self.state.yaw)

        # Save history
        self.x.append(self.state.x)
        self.y.append(self.state.y)
        self.t.append(t_sec)
        self.middle_axis_x_list.append(middle_axis_x)
        self.middle_axis_y_list.append(middle_axis_y)

        
        # Stanley steering control
        track_start_idx = self.path_splitting_indices[self.track_idx][0]
        track_end_idx = self.path_splitting_indices[self.track_idx][1]
        track_cx = self.cx[track_start_idx:track_end_idx + 1]
        track_cy = self.cy[track_start_idx:track_end_idx + 1]
        track_cyaw = self.cyaw[track_start_idx:track_end_idx + 1]
        di, track_target_idx = stanley_control(self.state, track_cx, track_cy, track_cyaw, self.forward)
        self.target_idx = track_target_idx + track_start_idx
        # print(f'cyaw [{self.target_idx}]:\t {self.cyaw[self.target_idx]*self.rad_to_deg}')
        print(f'self.target_idx:\t {self.target_idx}')
        print(f'self.track_end_idx:\t {track_end_idx}')

        # PID acceleration control
        if self.forward is True:
            ai = pid_control(self.target_speed_cm_s, self.state.v)      # ai = PID acceleration
        else:
            ai = pid_control(-self.target_speed_cm_s, self.state.v)      # ai = PID acceleration

        # Update state with motion model
        self.state.update(ai, di, dt)   # ai = acceleration, di = angle delta, dt = time delta

        # Save history
        self.yaw.append(self.state.yaw)
        self.v.append(self.state.v)

        if show_animation:  # pragma: no cover
            plt.cla()
            plt.plot(self.cx, self.cy, ".r", label="course")
            plt.plot(self.middle_axis_x_list, self.middle_axis_y_list, "-b", label="trajectory")
            plt.plot(self.cx[self.target_idx], self.cy[self.target_idx], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[cm/s]:" + str(self.state.v)[:4])
            plt.pause(0.1)

        self.idx += 1
        # Return needed yaw and velocity
        yaw_rad = self.state.yaw
        v_m_s = self.state.v / 100.0

        # Check for track end --> switch gear
        if track_target_idx >= track_end_idx-3:
            self.forward = not self.forward
            self.track_idx += 1
            print("Using next track going! Reverse Gear!")

        return yaw_rad, v_m_s