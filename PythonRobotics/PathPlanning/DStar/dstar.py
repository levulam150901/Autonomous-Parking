"""

D* grid planning

author: Nirnay Roy

See Wikipedia article (https://en.wikipedia.org/wiki/D*)

"""
import math

from sys import maxsize

import matplotlib.pyplot as plt

show_animation = False


class State:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.state = "."
        self.t = "new"  # tag for state
        self.h = 0
        self.k = 0

    def cost(self, state):
        if self.state == "#" or state.state == "#":
            return maxsize

        return math.sqrt(math.pow((self.x - state.x), 2) +
                         math.pow((self.y - state.y), 2))

    def set_state(self, state):
        """
        .: new
        #: obstacle
        e: oparent of current state
        *: closed state
        s: current state
        """
        if state not in ["s", ".", "#", "e", "*"]:
            return
        self.state = state


class Map:

    def __init__(self, ox_cm, oy_cm, start_cm, end_cm):
        # Ensure that within the algorithm implementation all coordinates
        # are indices in the grid and extend
        # from 0 to abs(<axis>_max - <axis>_min)
        self.x_min_world = int(min(ox_cm))
        self.y_min_world = int(min(oy_cm))

        self.x_max = int(abs(max(ox_cm) - self.x_min_world))
        self.y_max = int(abs(max(oy_cm) - self.y_min_world))

        start_cm_x =  start_cm[0] - self.x_min_world
        start_cm_y =  start_cm[1] - self.y_min_world
        end_cm_x = end_cm[0] - self.x_min_world
        end_cm_y = end_cm[1] - self.y_min_world

        self.row = self.y_max + 1
        self.col = self.x_max + 1
        self.map = self.init_map()

        self.start = self.map[start_cm_x][start_cm_y]
        self.end = self.map[end_cm_x][end_cm_y]

    def init_map(self):
        map_list = []
        for i in range(self.row):
            tmp = []
            for j in range(self.col):
                tmp.append(State(i, j))
            map_list.append(tmp)
        return map_list

    def get_x_position_list_without_offset(self, xlist):
        return [(x + self.x_min_world) for x in xlist]

    def get_y_position_list_without_offset(self, ylist):
        return [(y + self.y_min_world) for y in ylist]

    def get_neighbors(self, state):
        state_list = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i == 0 and j == 0:
                    continue
                if state.x + i < 0 or state.x + i >= self.row:
                    continue
                if state.y + j < 0 or state.y + j >= self.col:
                    continue
                state_list.append(self.map[state.x + i][state.y + j])
        return state_list

    def set_obstacle(self, point_list):
        # Ensure that within the algorithm implementation all coordinates
        # are indices in the grid and extend
        # from 0 to abs(<axis>_max - <axis>_min)
        point_list = [(x - self.x_min_world, y - self.y_min_world) for x, y in point_list]
        for x, y in point_list:
            if x < 0 or x >= self.row or y < 0 or y >= self.col:
                continue

            self.map[x][y].set_state("#")


class Dstar:
    def __init__(self, maps):
        self.map = maps
        self.open_list = set()

    def process_state(self):
        x = self.min_state()

        if x is None:
            return -1

        k_old = self.get_kmin()
        self.remove(x)

        if k_old < x.h:
            for y in self.map.get_neighbors(x):
                if y.h <= k_old and x.h > y.h + x.cost(y):
                    x.parent = y
                    x.h = y.h + x.cost(y)
        elif k_old == x.h:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y) \
                        or y.parent != x and y.h > x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
        else:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
                else:
                    if y.parent != x and y.h > x.h + x.cost(y):
                        self.insert(y, x.h)
                    else:
                        if y.parent != x and x.h > y.h + x.cost(y) \
                                and y.t == "close" and y.h > k_old:
                            self.insert(y, y.h)
        return self.get_kmin()

    def min_state(self):
        if not self.open_list:
            return None
        min_state = min(self.open_list, key=lambda x: x.k)
        return min_state

    def get_kmin(self):
        if not self.open_list:
            return -1
        k_min = min([x.k for x in self.open_list])
        return k_min

    def insert(self, state, h_new):
        if state.t == "new":
            state.k = h_new
        elif state.t == "open":
            state.k = min(state.k, h_new)
        elif state.t == "close":
            state.k = min(state.h, h_new)
        state.h = h_new
        state.t = "open"
        self.open_list.add(state)

    def remove(self, state):
        if state.t == "open":
            state.t = "close"
        self.open_list.remove(state)

    def modify_cost(self, x):
        if x.t == "close":
            self.insert(x, x.parent.h + x.cost(x.parent))

    def run(self, start, end):

        rx = []
        ry = []
        total_distance = 0.0
        self.insert(end, 0.0)

        while True:
            self.process_state()
            if start.t == "close":
                break

        start.set_state("s")
        s = start
        s = s.parent
        s.set_state("e")
        tmp = start

        while tmp != end:
            tmp.set_state("*")
            rx.append(tmp.x)
            ry.append(tmp.y)
            if tmp.parent.state == "#":
                self.modify(tmp)
                continue
            total_distance += tmp.cost(tmp.parent)
            tmp = tmp.parent
        tmp.set_state("e")

        return rx, ry, total_distance

    def modify(self, state):
        self.modify_cost(state)
        while True:
            k_min = self.process_state()
            if k_min >= state.h:
                break


def main():
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10)
    for i in range(-10, 60):
        ox.append(60)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60)
    for i in range(-10, 61):
        ox.append(-10)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40)
        oy.append(60 - i)

    start = [-5, -5]
    goal = [50, 50]

    m = Map(ox, oy, start, goal)

    print([(i, j) for i, j in zip(ox, oy)])
    m.set_obstacle([(i, j) for i, j in zip(ox, oy)])

    if show_animation:
        # start[0] = m.get_x_position_list_without_offset([start[0]])[0]
        # start[1] = m.get_y_position_list_without_offset([start[1]])[0]
        # goal[0] = m.get_x_position_list_without_offset([goal[0]])[0]
        # goal[1] = m.get_y_position_list_without_offset([goal[1]])[0]
        # ox = m.get_x_position_list_without_offset(ox)
        # oy = m.get_y_position_list_without_offset(oy)
        plt.plot(ox, oy, ".k")
        plt.plot(start[0], start[1], "og")
        plt.plot(goal[0], goal[1], "xb")
        plt.axis("equal")

    dstar = Dstar(m)
    rx, ry = dstar.run(m.start, m.end)


    if show_animation:
        rx = m.get_x_position_list_without_offset(rx)
        ry = m.get_y_position_list_without_offset(ry)
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    main()
