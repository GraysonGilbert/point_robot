"""This code is for the point robot and everything related to it. Including setting up obstacle space, map, and searching"""
import numpy as np
from collections import deque


"""Obstacle Definition:"""

class Map:
    def __init__(self, filename, map_width, map_height, clearance=2):

        self.filename = filename
        self.map_width = map_width
        self.map_height = map_height
        self.clearance = clearance


        self.grid = np.zeros((map_height, map_width), dtype=int)

        self.map_obstacles = []

    def read_obstacles(self):

        obstacles = []

        #self.grid[0, 1] = 1
        #print(self.grid)

        with open(self.filename, "r") as f:
            for line in f:
                shapes = line.strip().split(", ")
                obstacle_type = shapes[-1]

                if obstacle_type == "begin E":
                    x, y = map(int, (shapes[1:3]))
                    depth = int(shapes[4])
                    x, y, depth = x//10, y//10, depth//10   # Converting values to grid size
                    print(x, y, depth)
                    obstacles.append([x, y, depth])
                    self.e_obs_space(obstacles)

    """Defining E obstacle space using half-plane model."""
    def e_obs_space(self, obstacles):
        #Psuedo code:
        """
        for pt_x in range grid(x):
            if grid(x) - clearance <= pt_x <= grid(x) + clearance:
                for pt_y in range grid(y):
                    if grid(y) - clearance <= pt_y <= grid(y) + clearance:
                        "point is in obstacle space."
                        mark (pt_x, pt_y) in grid as -1
        """
        print(obstacles)
        x, y, depth = obstacles[0][:]
        
        x_width = x + 13
        y_depth = y + depth
        cutout_width, cutout_height = 8, 5
        cutout_x, cutout1_y = 20, 18
        cutout2_x, cutout2_y = 20, 28
    

        print(x, x_width, y, y_depth)

        for pt_x in range(self.map_width):
            if x - self.clearance <= pt_x <= x_width + self.clearance:
                for pt_y in range(self.map_height):
                    if y - self.clearance <= pt_y <= y_depth + self.clearance:
                        if cutout_x + self.clearance < pt_x <= cutout_x + self.clearance + cutout_width:
                            if cutout1_y + self.clearance < pt_y <= cutout1_y + cutout_height - self.clearance or cutout2_y + self.clearance < pt_y <= cutout2_y + cutout_height - self.clearance:
                                print("cutout e", pt_y)
                                continue
                            

                        #point is in obstacle space
                        self.grid[pt_x, pt_y] = -1
        
        """
        print(self.grid[13, 11])
        print(self.grid[21, 31])
        print(self.grid[22, 31])
        print(self.grid[23, 31])
        print(self.grid[29, 30])
        print(self.grid[30, 31])
        print(self.grid[31, 31])
        """

        

    





"""Creating Code to Run:"""
map_test = Map("map.txt", 180, 50)

map_test.read_obstacles()

start_point = input("Enter start position: ")
goal_point = input("Enter goal position: ")



#print(user_input)


