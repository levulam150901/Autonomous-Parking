"""

Path planning Sample Code with RRT with Reeds-Shepp path

author: AtsushiSakai(@Atsushi_twi)

"""
import copy
import math
import random
import sys
import pathlib
import matplotlib.pyplot as plt
import numpy as np
from shapely import Polygon

sys.path.append(str(pathlib.Path(__file__).parent.parent))

from ReedsSheppPath import reeds_shepp_path_planning
from RRTStar.rrt_star import RRTStar

show_animation = True


class RRTStarReedsShepp(RRTStar):
    """
    Class for RRT star planning with Reeds Shepp path
    """

    class Node(RRTStar.Node):
        """
        RRT Node
        """

        def __init__(self, x, y, yaw, robot_width, robot_length):
            super().__init__(x, y)
            self.yaw = yaw
            self.path_yaw = []
            self.robot_width = robot_width
            self.robot_length = robot_length

    def __init__(self, start, goal, obstacle_list, rand_area,
                 max_iter=200, step_size=0.2,
                 connect_circle_dist=50.0, curvature=3.0,
                 robot_width=0.0, robot_length=0.0):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        robot_radius: robot body modeled as circle with given radius

        """
        self.start = self.Node(start[0], start[1], start[2], robot_width, robot_length)
        self.end = self.Node(goal[0], goal[1], goal[2], robot_width, robot_length)
        self.min_rand_x = rand_area[0]
        self.max_rand_x = rand_area[1]
        self.min_rand_y = rand_area[2]
        self.max_rand_y = rand_area[3]
        self.max_iter = max_iter
        self.step_size = step_size
        self.obstacle_list = obstacle_list
        self.connect_circle_dist = connect_circle_dist
        self.robot_width = robot_width
        self.robot_length = robot_length
        self.obstacle_rect_list = []

        for (ox, oy, width, length, rad) in obstacle_list:
            # Calculate coordinates of all four corners of the robot for each point in the path]
            x = ox - width / 2.0
            y = oy - length / 2.0
            xy = (x, y)
            obst_corners = plt.Rectangle(xy, width, length, angle=np.rad2deg(rad),
                                         rotation_point=(ox, oy)).get_corners()
            self.obstacle_rect_list.append(Polygon(obst_corners))
                
        self.curvature = curvature                     # Defines max steering angle
        self.goal_yaw_th = np.deg2rad(1.0)
        self.goal_xy_th = 0.5

    def set_random_seed(self, seed):
        random.seed(seed)

    def planning(self, animation=True, search_until_max_iter=True):
        """
        planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd)

            if self.check_collision(self,
                    new_node, self.obstacle_list, self.robot_width, self.robot_length):
                near_indexes = self.find_near_nodes(new_node)
                new_node = self.choose_parent(new_node, near_indexes)
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_indexes)
                    self.try_goal_path(new_node)

            if animation and i % 5 == 0:
                self.plot_start_goal_arrow()
                self.draw_graph(rnd)

            if (not search_until_max_iter) and new_node:  # check reaching the goal
                last_index = self.search_best_goal_node()
                if last_index:
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index:
            return self.generate_final_course(last_index)
        else:
            print("Cannot find path")

        return None

    def try_goal_path(self, node):

        goal = self.Node(self.end.x, self.end.y, self.end.yaw, self.robot_width, self.robot_length)

        new_node = self.steer(node, goal)
        if new_node is None:
            return

        if self.check_collision(self,
                new_node, self.obstacle_list, self.robot_width, self.robot_length):
            self.node_list.append(new_node)

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                # self.plot_robot(node.x, node.y, node.yaw, node.robot_width, node.robot_length)
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, width, length, rad) in self.obstacle_list:
            self.plot_rectangle(ox, oy, width, length, rad)
            # print("Width", width)
            # print("Length", length)

        self.plot_robot(self.start.x, self.start.y, self.start.yaw, self.start.robot_width, self.start.robot_length)
        self.plot_robot(self.end.x, self.end.y, self.end.yaw, self.end.robot_width, self.end.robot_length)
        plt.axis('auto')
        plt.grid(True)
        self.plot_start_goal_arrow()
        plt.pause(0.01)

    def plot_rectangle(self, x, y, width, height, rad):
        rectangle = plt.Rectangle((x - width / 2, y - height / 2), width, height, angle=np.rad2deg(rad),
                                         rotation_point=(x, y), color='k', fill=True)
        plt.gca().add_patch(rectangle)

    def plot_robot(self, x, y, yaw, robot_width, robot_length):
        corners = self.calculate_robot_corners(x, y, yaw, robot_width, robot_length)
        x = [corner[0] for corner in corners] + [corners[0][0]]
        y = [corner[1] for corner in corners] + [corners[0][1]]
        plt.plot(x, y, "-b")

    def calculate_robot_corners(self, x, y, yaw, robot_width, robot_length):
        c, s = math.cos(yaw), math.sin(yaw)
        # Define the four corners of the rectangle (in the body frame)
        corners = [(-robot_length / 2, -robot_width / 2), (-robot_length / 2, robot_width / 2),
                   (robot_length / 2, robot_width / 2), (robot_length / 2, -robot_width / 2)]
        # Rotate the corners to the global frame
        rotated_corners = [(x + c * x0 - s * y0, y + s * x0 + c * y0) for (x0, y0) in corners]
        return rotated_corners
    @staticmethod
    def check_collision(self, node, obstacle_list, robot_width, robot_length):
        if node is None:
            return False
        # Check collision for each node on the path to the new node
        for idx in range(len(node.path_x)):
            # Calculate coordinates of all four corners of the robot for the new path node 
            x1y1 = (node.path_x[idx] - robot_width, node.path_y[idx] - robot_length)
            corners = plt.Rectangle(x1y1, robot_width * 2, robot_length * 2, angle=np.rad2deg(node.path_yaw[idx]),
                                    rotation_point=(node.path_x[idx], node.path_y[idx])).get_corners()
            robot_rect = Polygon(corners)

            for obstacle_rect in self.obstacle_rect_list:
                if obstacle_rect.intersects(robot_rect):
                    return False  # collision

        return True  # safe

    def plot_start_goal_arrow(self):
        reeds_shepp_path_planning.plot_arrow(
            self.start.x, self.start.y, self.start.yaw)
        reeds_shepp_path_planning.plot_arrow(
            self.end.x, self.end.y, self.end.yaw)

    def steer(self, from_node, to_node):

        px, py, pyaw, mode, course_lengths = reeds_shepp_path_planning.reeds_shepp_path_planning(
            from_node.x, from_node.y, from_node.yaw, to_node.x,
            to_node.y, to_node.yaw, self.curvature, self.step_size)

        if not px:
            return None

        new_node = copy.deepcopy(from_node)
        new_node.x = px[-1]
        new_node.y = py[-1]
        new_node.yaw = pyaw[-1]

        new_node.path_x = px
        new_node.path_y = py
        new_node.path_yaw = pyaw
        new_node.cost += sum([abs(l) for l in course_lengths])
        new_node.parent = from_node

        return new_node

    def calc_new_cost(self, from_node, to_node):

        _, _, _, _, course_lengths = reeds_shepp_path_planning.reeds_shepp_path_planning(
            from_node.x, from_node.y, from_node.yaw, to_node.x,
            to_node.y, to_node.yaw, self.curvature, self.step_size)
        if not course_lengths:
            return float("inf")

        return from_node.cost + sum([abs(l) for l in course_lengths])

    def get_random_node(self):

        rnd = self.Node(random.uniform(self.min_rand_x, self.max_rand_x),
                        random.uniform(self.min_rand_y, self.max_rand_y),
                        random.uniform(-math.pi, math.pi),
                        self.robot_width,
                        self.robot_length
                        )

        return rnd

    def search_best_goal_node(self):

        goal_indexes = []
        for (i, node) in enumerate(self.node_list):
            if self.calc_dist_to_goal(node.x, node.y) <= self.goal_xy_th:
                goal_indexes.append(i)
        print("goal_indexes:", len(goal_indexes))

        # angle check
        final_goal_indexes = []
        for i in goal_indexes:
            if abs(self.node_list[i].yaw - self.end.yaw) <= self.goal_yaw_th:
                final_goal_indexes.append(i)

        print("final_goal_indexes:", len(final_goal_indexes))

        if not final_goal_indexes:
            return None

        min_cost = min([self.node_list[i].cost for i in final_goal_indexes])
        print("min_cost:", min_cost)
        for i in final_goal_indexes:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def generate_final_course(self, goal_index):
        path = [[self.end.x, self.end.y, self.end.yaw]]
        node = self.node_list[goal_index]
        path_cost = 0.0
        while node.parent:
            for (ix, iy, iyaw) in zip(reversed(node.path_x), reversed(node.path_y), reversed(node.path_yaw)):
                path.append([ix, iy, iyaw])
            path_cost += node.cost
            node = node.parent
        path.append([self.start.x, self.start.y, self.start.yaw])
        return path, path_cost


def main(max_iter=100):
    print("Start " + __file__)

    # ====Search Path with RRT====
    obstacleList = [
        (5, 5, 1, 1, 0),  # Rectangle with width=1 and height=1
        (4, 6, 1, 1, 0),
        (4, 8, 1, 1, 0),
        (4, 10, 1, 1, 0),
        (6, 5, 1, 1, 0),
        (7, 5, 1, 1, 0),
        (8, 6, 1, 1, 0),
        (8, 8, 1, 1, 0),
        (8, 10, 1, 1, 0)
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    start = [0.0, 0.0, np.deg2rad(0.0)]
    goal = [6.0, 7.0, np.deg2rad(90.0)]
    robot_width = 1.3
    robot_length = 1.7
    min_x = min(start[0], goal[0])
    max_x = max(start[0], goal[0])
    min_y = min(start[1], goal[1])
    max_y = max(start[1], goal[1])
    threshold_m = 0.5
    rand_area_x_m = [min_x - threshold_m, max_x + threshold_m]
    rand_area_y_m = [min_y - threshold_m, max_y + threshold_m]

    # Define path planning parameters
    rand_area = [rand_area_x_m[0], rand_area_x_m[1], rand_area_y_m[0], rand_area_y_m[1]]
    rrt_star_reeds_shepp = RRTStarReedsShepp(start, goal,
                                             obstacleList,
                                             rand_area=rand_area,
                                             max_iter=max_iter,
                                             robot_width=robot_width,
                                             robot_length=robot_length)
    path = rrt_star_reeds_shepp.planning(animation=show_animation)

    # Draw final path
    if path and show_animation:  # pragma: no cover
        rrt_star_reeds_shepp.draw_graph()
        plt.plot([x for (x, y, yaw) in path], [y for (x, y, yaw) in path], '-r')
        plt.grid(True)
        plt.pause(0.1)
        plt.show()


if __name__ == '__main__':
    main()
