"""

Path tracking simulation with Stanley steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

Ref:
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

"""
import numpy as np
import matplotlib.pyplot as plt
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

from PathPlanning.CubicSpline import cubic_spline_planner

# -- Parameters for v = 0.05 [m/s] -- #
kd = 0.5  # cross track error control gain
ke = 4.0  # heading error control gain

# -- Parameters for v = 0.10 [m/s] -- #
# kd = 0.05  # cross track error control gain
# ke = 2.0  # heading error control gain

Kp = 0.5  # speed proportional gain
dt = 0.1  # [s] time difference
L = 11.0/2.0  # [cm] Wheel base of vehicle RVR

show_animation = True


class State(object):
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, acceleration, delta, dt=dt):
        """
        Update the state of the vehicle.

        Stanley Control using own RVR Motion Model

        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """
        # Own RVR motion model:
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v * delta * dt    
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt


def pid_control(target, current):
    """
    Proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return Kp * (target - current)


def stanley_control(state, cx, cy, cyaw, forward):
    """
    Stanley steering control.

    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param track_idx: (int)
    :return: (float, int)
    """
    # calculate current target index of spline curve and error of RVR middle axle to it.
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    # never get below the last target index
    # if last_target_idx >= current_target_idx:
    #     current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # print(f'heading error theta_e [Deg]:\t{np.rad2deg(theta_e)}')

    # theta_d corrects the cross track error
    if forward is not True:
        error_front_axle *= -1
    theta_d = error_front_axle
    # print(f'track error theta_d [Deg]:\t{np.rad2deg(theta_d)}')

    # Steering control
    delta = (ke * theta_e + kd * theta_d)
    # print(f'Delta [Deg]:\t\t\t{np.rad2deg(delta)}')

    return delta, current_target_idx


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calculate RVR middel axis position
    fx = state.x + L * np.cos(state.yaw) 
    fy = state.y + L * np.sin(state.yaw)

    axle_vec = [-np.cos(state.yaw + np.pi / 2),
                -np.sin(state.yaw + np.pi / 2)]

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    
    # Find the indices of the minimum values
    min_indices = np.where(d == np.min(d))[0]
    
    # Choose the highest index among the minimum values
    target_idx = max(min_indices)

    # Project RMS error onto front axle vector
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], axle_vec)

    return target_idx, error_front_axle


def main():
    """Plot an example of Stanley steering control on a cubic spline."""
    #  target course
    ax = [0.0, 100.0, 100.0, 50.0, 60.0]
    ay = [0.0, 0.0, -30.0, -20.0, 0.0]

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)

    target_speed = 30.0 / 3.6  # [m/s]

    max_simulation_time = 100.0

    # Initial state
    state = State(x=-0.0, y=5.0, yaw=np.radians(20.0), v=0.0)

    last_idx = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_idx, _ = calc_target_index(state, cx, cy)

    while max_simulation_time >= time and last_idx > target_idx:
        ai = pid_control(target_speed, state.v)
        di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
        state.update(ai, di)

        time += dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
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


if __name__ == '__main__':
    main()
