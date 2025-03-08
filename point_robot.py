"""This code is for the point robot and everything related to it. Including setting up obstacle space, map, and searching"""
import numpy as np
from collections import deque


"""Obstacle Definition:"""

class Map:
    def __init__(self, filename, map_width, map_height, clearance=2):

        self.map_utils = MapUtils()

        self.filename = filename
        self.map_width = map_width
        self.map_height = map_height
        self.clearance = clearance


        self.grid = np.zeros((map_height, map_width), dtype=int)

        self.map_obstacles = []
    
    """Checks to see if a start point is valid."""
    def check_start_position(self, start_pt):
        if 0 <= start_pt[0] < self.map_width and 0 <= start_pt[1] < self.map_height: # Checks to see if start point is within bounds of map
            if self.grid[start_pt[0], start_pt[1]] != -1:
                print("Valid start point!")
                return True
            else:
                print("Start point located in obstacle! Try again.")
                return False
        else:
            print("Start point out of map boundary! Try again.")
            return False
    
    """Checks to see if a goal point is valid."""
    def check_goal_position(self, goal_pt):
        if 0 <= goal_pt[0] < self.map_width and 0 <= goal_pt[1] < self.map_height: # Checks to see if start point is within bounds of map
            if self.grid[goal_pt[0], goal_pt[1]] != -1:
                print("Valid goal point!")
                return True
            else:
                print("Goal point located in obstacle! Try again.")
                return False
        else:
            print("Goal point out of map boundary! Try again.")
            return False

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
                    print("e_obs_space", x, y, depth)
                    obstacles.append([x, y, depth])
                    self.e_obs_space(obstacles)

                if obstacle_type == "begin N":
                    x, y = map(int, (shapes[1:3]))
                    depth = int(shapes[4])
                    x, y, depth = x//10, y//10, depth//10   # Converting values to grid size
                    print("n_obs_sapce", x, y, depth)
                    obstacles.append([x, y, depth])
                    self.n_obs_space(obstacles)
                
                if obstacle_type == "begin P":
                    x, y = map(int, (shapes[1:3]))
                    depth = int(shapes[4])
                    x, y, depth = x//10, y//10, depth//10   # Converting values to grid size
                    print("p_obs_sapce", x, y, depth)
                    obstacles.append([x, y, depth])
                    self.p_obs_space(obstacles)

                if obstacle_type == "begin M":
                    x, y = map(int, (shapes[1:3]))
                    depth = int(shapes[4])
                    x, y, depth = x//10, y//10, depth//10   # Converting values to grid size
                    print("m_obs_sapce", x, y, depth)
                    obstacles.append([x, y, depth])
                    self.m_obs_space(obstacles)

                if obstacle_type == "begin 6":
                    x, y = map(int, (shapes[1:3]))
                    depth = int(shapes[4])
                    x, y, depth = x//10, y//10, depth//10   # Converting values to grid size
                    print("six_obs_sapce", x, y, depth)
                    obstacles.append([x, y, depth])
                    self.six_obs_space(obstacles)

                if obstacle_type == "begin 1":
                    x, y = map(int, (shapes[1:3]))
                    depth = int(shapes[4])
                    x, y, depth = x//10, y//10, depth//10   # Converting values to grid size
                    print("one_obs_sapce", x, y, depth)
                    obstacles.append([x, y, depth])
                    self.one_obs_space(obstacles)

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
        #print(obstacles)
        x, y, depth = obstacles[-1][:]
        
        x_width = x + 13
        y_depth = y + depth
        cutout_width, cutout_height = 8, 5
        cutout_x, cutout1_y = 20, 18
        cutout2_x, cutout2_y = 20, 28
    

        #print(x, x_width, y, y_depth)

        for pt_x in range(self.map_width):
            if x - self.clearance <= pt_x < x_width + self.clearance:
                for pt_y in range(self.map_height):
                    if y - self.clearance <= pt_y < y_depth + self.clearance:
                        if cutout_x + self.clearance <= pt_x < cutout_x + self.clearance + cutout_width:
                            if cutout1_y + self.clearance <= pt_y < cutout1_y + cutout_height - self.clearance or cutout2_y + self.clearance <= pt_y < cutout2_y + cutout_height - self.clearance:
                                #print("cutout e", pt_y)
                                continue
                            

                        #point is in obstacle space
                        self.grid[pt_y, pt_x] = -1
        
    def n_obs_space(self, obstacles):

        x, y, depth = obstacles[-1][:]
        x_width = x + 15
        y_depth = y + depth


        for pt_x in range(self.map_width):
            if x - self.clearance <= pt_x < x_width + self.clearance:
                for pt_y in range(self.map_height):
                    if y - self.clearance <= pt_y < y_depth + self.clearance:
                        
                        self.grid[pt_y, pt_x] = -1
                        """NOT DONE YET"""




    def p_obs_space(self, obstacles):

        x, y, depth = obstacles[-1][:]
        x_width = x + 5
        y_depth = y + depth
        circ_x = x + 5
        circ_y = y + 5
        circ_r = 5

        for pt_x in range(self.map_width):
            if x - self.clearance <= pt_x < x_width + self.clearance:
                for pt_y in range(self.map_height):
                    if y - self.clearance <= pt_y < y_depth + self.clearance:
                        self.grid[pt_y, pt_x] = -1
        for pt_x in range(self.map_width):
            if x - self.clearance <= pt_x < x_width + circ_r + self.clearance:
                for pt_y in range(self.map_height):
                    if (pt_x - circ_x) ** 2 + (pt_y - circ_y) ** 2 <= (circ_r + self.clearance) ** 2:
                        self.grid[pt_y, pt_x] = -1
                        

    def m_obs_space(self, obstacles):
        x, y, depth = obstacles[-1][:]
        x_width = x + 25
        y_depth = y + depth


        for pt_x in range(self.map_width):
            if x - self.clearance <= pt_x < x_width + self.clearance:
                for pt_y in range(self.map_height):
                    if y - self.clearance <= pt_y < y_depth + self.clearance:
                        
                        self.grid[pt_y, pt_x] = -1
                        """NOT DONE YET"""
                    

    def six_obs_space(self, obstacles):
        x, y, depth = obstacles[-1][:]
        x_width1 = x + 5
        x_width2 = x + 13
        y_depth = y + depth
        circ_r = 9
        circ_x = x + circ_r
        circ_y = y_depth
        
        for pt_x in range(self.map_width):
            if x - self.clearance <= pt_x < x_width1 + self.clearance:
                for pt_y in range(self.map_height):
                    if y - self.clearance <= pt_y < y_depth + self.clearance:
                        self.grid[pt_y, pt_x] = -1

        for pt_x in range(self.map_width):
            if x - self.clearance <= pt_x < x_width2 + self.clearance:
                for pt_y in range(self.map_height):
                    if y - self.clearance <= pt_y < y_depth + self.clearance:
                        self.grid[pt_y, pt_x] = -1
                    
        for pt_x in range(self.map_width):
            if x - self.clearance <= pt_x < x + (2 * circ_r) + self.clearance:
                for pt_y in range(self.map_height):
                    if (pt_x - circ_x) ** 2 + (pt_y - circ_y) ** 2 <= (circ_r + self.clearance) ** 2:
                        self.grid[pt_y, pt_x] = -1

    def one_obs_space(self, obstacles):

        x, y, depth = obstacles[-1][:]
        x_width = x + 5
        y_depth = y + depth

        for pt_x in range(self.map_width):
            if x - self.clearance <= pt_x < x_width + self.clearance:
                for pt_y in range(self.map_height):
                    if y - self.clearance <= pt_y < y_depth + self.clearance:
                        
                        self.grid[pt_y, pt_x] = -1
                        

class MapUtils:
    def __init__(self):
        pass

    def write_obstacle_file(self, grid):
        
        with open("obstacle_output_file.txt", "w") as f:
            for row in grid:
                f.write(" ".join(map(str, row)) + "\n")
        
        

    





"""Creating Code to Run:"""
map_test = Map("map.txt", 180, 50)

map_test.read_obstacles()

#map_utils = MapUtils()
map_test.map_utils.write_obstacle_file(map_test.grid)

valid_start = False
valid_goal = False

"""
while valid_start != True:

    start_point = input("Enter start position as x, y: ")
    start_point = list(map(int, start_point.split(", ")))
    valid_start = map_test.check_start_position(start_point)



while valid_goal != True:

    goal_point = input("Enter goal position as x, y: ")
    goal_point = list(map(int, goal_point.split(", ")))
    #print(goal_point)
    valid_goal = map_test.check_goal_position(goal_point)
    
#print(start_point)

"""








