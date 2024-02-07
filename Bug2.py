#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'ar1'

from DyorBase import *

debug = False
class Bug2(DyorBase):
    def __init__(self, target_name='target', bot_name='Bot', wheel_speed=1.0):
        DyorBase.__init__(self, target_name, bot_name, wheel_speed)
        self.start_target_pos = None
        self.start_bot_pos = None
        self.about = "Algorithm Bug2"
        self.distance_hitpoint_to_target = None
        #self.last_state = None


        self.stop_move()

        self.state = States.ROTATING_TO_GOAL


        self.print_about_info()

    def loop(self):
        # debug = True
        debug = False
        if debug:
            defaultIdleFps = self.sim.getInt32Param(self.sim.intparam_idle_fps)
            self.sim.setInt32Param(self.sim.intparam_idle_fps, 0)
            self.client.setStepping(True)

        # Start the Simulation here, or if commented out in CopelliaSim
        self.stop_move()    # Make sure velocities are set to 0
        self.sim.startSimulation()
        print("Simulation started")
        finished = False
        self._init_values()
        self.start_target_pos = self.target_pos

        # Receive obstacles from scene
        self._init_obstacle_handle()

        path_planning_done_once = False
        # Path Following
        while not finished:
            if debug:
                self.client.step()

            # Read data from bot
            self.read_values()
            # self.read_US_sensor()
            # self.read_LIDAR_sensors()
            
            # Only 2D
            self.target_pos.z = self.bot_pos.z = 0.0
            
            # Init bot starting position and goal position once
            if self.start_bot_pos is None:
                self.start_bot_pos = self.bot_pos
            if self.start_target_pos is None:
                self.start_target_pos = self.target_pos

            # Get current bot direction
            q_rot = Quaternion()
            q_rot.set_from_vector(self.bot_euler_angles.z, Vector3(0.0, 0.0, 1.0))
            self.bot_dir = q_rot.rotate(Vector3(1.0, 0.0, 0.0))

            # PathPlanning
            if path_planning_done_once is False:
                rx_m, ry_m, ryaw_rad = self.dyor_path_planning()
                path_planning_done_once = True
            # rx_m, ry_m, ryaw_rad = zip(*path)

            # Plot arrows
            # if doneOnce <= 10:
            #     for x, y, raw in path:
            #         reeds_shepp_path_planning.plot_arrow(x*100.0, y*100.0, raw, length=5, width=0.5)
            #     doneOnce += 1
            #     plt.show()
                
            # PathTracking
            yaw_rad, v_m_s = self.dyor_path_tracking(rx_m, ry_m, ryaw_rad)

            # Direction Controller 
            q_rot.set_from_vector(yaw_rad, Vector3(0.0, 0.0, 1.0))
            self.stanley_dir = q_rot.rotate(Vector3(1.0, 0.0, 0.0))
            self.stanley_dir_euler_angle_z = math.atan2(self.stanley_dir.y, self.stanley_dir.x)
            steering_angle_rad = angle_between_vectors(self.bot_dir, self.stanley_dir)
            # print(f'bot_euler_angle [Deg]: {round(self.bot_euler_angles.z*self.rad_to_deg)}\t\
            #       stanley_dir_euler_angle [Deg]:\t {round(self.stanley_dir_euler_angle_z*self.rad_to_deg)}\t\
            #         steering_angle [Deg]:\t {round(steering_angle_rad*self.rad_to_deg)}')

            # Velocity Controller 
            wheel_radius_m = 0.07 / 2.0                                     # RVR Robot [m]
            v_deg_per_sec = (v_m_s/ (wheel_radius_m * math.pi)) * 180.0     # Angular velocity [deg/s]

            # Variant B - Linear steering
            v_left_deg_per_sec = v_deg_per_sec - steering_angle_rad/(np.pi/2) * v_deg_per_sec
            v_right_deg_per_sec = v_deg_per_sec + steering_angle_rad/(np.pi/2) * v_deg_per_sec

            self.sim.setJointTargetVelocity(self.left_front_motor_handle, v_left_deg_per_sec / 57.29578)    # Velocity 1.0 equals 57.29578 deg/s in Coppeliasim
            self.sim.setJointTargetVelocity(self.left_rear_motor_handle, v_left_deg_per_sec / 57.29578)    # Velocity 1.0 equals 57.29578 deg/s in Coppeliasim
            self.sim.setJointTargetVelocity(self.right_front_motor_handle, v_right_deg_per_sec / 57.29578)  # Velocity 1.0 equals 57.29578 deg/s in Coppeliasim
            self.sim.setJointTargetVelocity(self.right_rear_motor_handle, v_right_deg_per_sec / 57.29578)  # Velocity 1.0 equals 57.29578 deg/s in Coppeliasim

            # Save new bot velocities [m/s]
            self.bot_left_vel_m_s = v_left_deg_per_sec / 180.0 * wheel_radius_m * math.pi
            self.bot_right_vel_m_s = v_right_deg_per_sec / 180.0 * wheel_radius_m * math.pi

            # Simulation Exit Condition
            if distance_between_points(self.bot_pos, self.target_pos) < 0.03:
                self.stop_move()
                print("Reached parking position successfully! \nExit Simulation.")
                time.sleep(10)
                self.sim.stopSimulation()
                finished = True
            


    ##################################################################################################
    def action_moving(self):
        # drive forward
        wheel_speed_left  = self.vref/self.wheel_radius
        wheel_speed_right = self.vref/self.wheel_radius

        self.sim.setJointTargetVelocity(self.left_front_motor_handle, wheel_speed_left)
        self.sim.setJointTargetVelocity(self.left_rear_motor_handle, wheel_speed_left)
        self.sim.setJointTargetVelocity(self.right_front_motor_handle, wheel_speed_right)
        self.sim.setJointTargetVelocity(self.right_rear_motor_handle, wheel_speed_right)

##################################################################################################
    def action_rotating_to_goal(self):
        angle = Utils.angle_between_vectors(self.bot_dir, self.target_dir)
        vel = angle
        # clockwise rotation
        self.sim.setJointTargetVelocity(self.left_front_motor_handle, vel)
        self.sim.setJointTargetVelocity(self.left_rear_motor_handle, vel)
        self.sim.setJointTargetVelocity(self.right_front_motor_handle, -vel)
        self.sim.setJointTargetVelocity(self.right_rear_motor_handle, -vel)

##################################################################################################
    def action_rotating_left(self):
        angle = Utils.angle_between_vectors(self.bot_dir, self.left_dir)    # 90 Degree to left
        vel = angle
        # clockwise rotation
        self.sim.setJointTargetVelocity(self.left_front_motor_handle, vel)
        self.sim.setJointTargetVelocity(self.left_rear_motor_handle, vel)
        self.sim.setJointTargetVelocity(self.right_front_motor_handle, -vel)
        self.sim.setJointTargetVelocity(self.right_rear_motor_handle, -vel)

##################################################################################################
    def action_rotating_clockwise(self):
        vel = 0.3 * self.vref / self.wheel_radius  
        # clockwise rotation
        self.sim.setJointTargetVelocity(self.left_front_motor_handle, vel)
        self.sim.setJointTargetVelocity(self.left_rear_motor_handle, vel)
        self.sim.setJointTargetVelocity(self.right_front_motor_handle, -vel)
        self.sim.setJointTargetVelocity(self.right_rear_motor_handle, -vel)

##################################################################################################
    def action_rotating_anticlockwise(self):
        vel = 0.3 * self.vref / self.wheel_radius
        # clockwise rotation
        self.sim.setJointTargetVelocity(self.left_front_motor_handle, -vel)
        self.sim.setJointTargetVelocity(self.left_rear_motor_handle, -vel)
        self.sim.setJointTargetVelocity(self.right_front_motor_handle, vel)
        self.sim.setJointTargetVelocity(self.right_rear_motor_handle, vel)

##################################################################################################
    def action_rounding(self):
        ratio = self.d_FR / self.d_RR  # difference between front right and rear right sensor
        
        # Hier LIDAR verwenden um das rounding zu realisieren
        if ratio > 1.02 and self.d_FR > self.lidar_dist_thresh:                # Drehe nach Rechts, wenn Abstand_Vorne_Rechts > Abstand_Hinten_Rechts und absoluter Abstand größer als 0.25m
            self.action_rotating_clockwise()    
            print("ROTATION CLOCKWISE")
        elif ratio < 0.98 and self.d_FR > self.lidar_dist_thresh:              # Drehe nach Links, wenn Abstand_Vorne_Rechts < Abstand_Hinten_Rechts und absoluter Abstand größer als 0.25m
            self.action_rotating_anticlockwise()
            print("ROTATION ANTICLOCKWISE")
        else:                           # Vorwärtsfahren, wenn Abstand_Vorne_Rechts == Abstand_Hinten_Rechts
            self.action_moving()
            print("ACTION MOVING")
    

########################################################################################################################
# checks, whether robot is on m-line
    def is_bot_on_mline(self):
        # (x-x1)/(x2-x1) = (y-y1)/(y2-y1).
        diff_x = (self.bot_pos.x - self.start_bot_pos.x) / (self.start_target_pos.x - self.start_bot_pos.x)
        diff_y = (self.bot_pos.y - self.start_bot_pos.y) / (self.start_target_pos.y - self.start_bot_pos.y)
        delta = 0.01
        if diff_x - delta < diff_y < diff_x + delta:
            return True
        return False
    
########################################################################################################################
# bot pointing to goal
    def is_bot_pointing_to(self, dir):
        # Winkel zwischen bot_dir und target_dir
        angle = Utils.angle_between_vectors(self.bot_dir, dir)

        # Angle smaller than 1 degree
        if abs(angle) < (1.0*self.deg_to_rad):
            return True
        else:
            return False