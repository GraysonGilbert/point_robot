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
        self.flipped_grid = np.flipud(self.grid)

        self.map_obstacles = []
    

        
    """Checks to see if a move is valid."""
    def is_move_valid(self, state):
        pass

    """Reads obstacles from map.txt to construct grid of obstacle space."""
    def read_obstacles(self):

        obstacles = []

        #self.grid[0, 1] = 1
        #print(self.grid)
        self.boundary_obs_space()

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

        self.flip_grid()

    """Defining boundary obstacle space using array slicing."""
    def boundary_obs_space(self):

        self.grid[:2, :] = -1
        self.grid[-2:, :] = -1
        self.grid[:, :2] = -1
        self.grid[:, -2:] = -1

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
        cutout2_y = 28

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

    """Defining N obstacle space using half-plane model."""
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

    """Defining P obstacle space using half-plane model and semi-algebraic model."""
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
                        
    """Defining M obstacle space using half-plane model."""
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
                    
    """Defining 6 obstacle space using half-plane model and semi-algebraic model."""
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

    """Defining 1 obstacle space using half-plane model."""
    def one_obs_space(self, obstacles):

        x, y, depth = obstacles[-1][:]
        x_width = x + 5
        y_depth = y + depth

        for pt_x in range(self.map_width):
            if x - self.clearance <= pt_x < x_width + self.clearance:
                for pt_y in range(self.map_height):
                    if y - self.clearance <= pt_y < y_depth + self.clearance:
                        
                        self.grid[pt_y, pt_x] = -1
    
    def flip_grid(self):
        self.flipped_grid = np.flipud(self.grid)


"""This class handles the writing of the obstacle information file to generate the map."""
class MapUtils:
    def __init__(self):
        pass
    
    """Writes obstacle file for map."""
    def write_obstacle_file(self, grid):
        
        with open("obstacle_output_file.txt", "w") as f:
            for row in grid:
                f.write(" ".join(map(str, row)) + "\n")

    """Writes obstacle file for map."""
    def write_flipped_obstacle_file(self, grid):

        flipped_grid = np.flipud(grid)
        
        with open("flipped_obstacle_output_file.txt", "w") as f:
            for row in flipped_grid:
                f.write(" ".join(map(str, row)) + "\n")

"""This class handles all the actions related gather the robot state in the map."""
class PointRobot:
    def __init__(self, node_state_i=np.array([0,0]), node_index_i=0, parent_node_index_i=None):

        self.map = Map("map.txt", 180, 50)

        self.node_index_i = node_index_i
        self.parent_node_index_i = parent_node_index_i
        self.node_state_i = node_state_i

        self.state = self.get_state()
        self.possible_moves = self.get_possible_moves()
        
    
    """Allows for comparing robot state to determine if they are equal to one another."""
    def __eq__(self, other):
        return np.array_equal(self.node_state_i, other.node_state_i)
    
    """Creates a hash value of the robot state based on its node index. Required to efficiently track visited states."""
    def __hash__(self):
        return hash(tuple(self.node_state_i))

    def get_start_position(self, valid_start=False):
        while valid_start != True:

            start_point = input("Enter start position as x, y: ")
            start_point = list(map(int, start_point.split(", ")))
            valid_start = self.check_start_position(start_point)
            self.possible_moves = self.get_possible_moves()
    
    """Checks to see if a start point is valid."""
    def check_start_position(self, start_pt):
        if 0 <= start_pt[0] < self.map.map_width and 0 <= start_pt[1] < self.map.map_height: # Checks to see if start point is within bounds of map
            if self.map.grid[start_pt[0], start_pt[1]] != -1:
                
                self.node_state_i = np.array(start_pt)
                print("Valid start point!")
                return True
            else:
                print("Start point located in obstacle! Try again.")
                return False
        else:
            print("Start point out of map boundary! Try again.")
            return False

    """MAY NOT NEED."""
    def convert_row_state(self):
        state = self. node_state_i[-1]
        row = state[0]
        row = abs(self.map.map_height - row)
        print(row)


    """MAY NOT NEED THIS"""
    """Determine the current state of the robot in the map."""
    def get_state(self):

        state = self.node_state_i
        
        row = state[0]
        col =  state[1]
        #print(row, col)
        
        return row, col


    """Find all possible moves based on current robot state."""
    def get_possible_moves(self):
        
        #row, col = 3, 3
        row, col = self.get_state()
        
        moves = {}

        move_options = {
            "up": (1, 0), # May need some adjusting
            "down": (-1, 0),
            "left": (0, -1),
            "right": (0, 1),
            "up_left": (1, -1),
            "up_right": (1, 1),
            "down_left": (-1, -1),
            "down_right": (-1, 1),
        }
        
        for move, (row_move, col_move) in move_options.items(): # Checks that a move is valid, meaning it stays within the bounds of the 3x3 puzzle
            new_row, new_col = row + row_move, col + col_move
            
            if self.map.flipped_grid[new_row, new_col] != -1:
                moves[move] = (row_move, col_move)
        #print(moves)
        return moves

    def change_state(self, current_row, current_col, new_row, new_col):
        new_state = self.node_state_i.copy()

        index_1 = current_col
        
    """Moves robot state up by 1 position."""
    def move_up(self):

        if "up" in self.possible_moves:
            row_change, col_change = self.possible_moves["up"]
            current_row, current_col = self.get_state()
            new_row, new_col = current_row + row_change, current_col + col_change

            return [new_row, new_col]
        
        return None
    
    """Moves robot state down by 1 position."""
    def move_down(self):

        if "down" in self.possible_moves:
            row_change, col_change = self.possible_moves["down"]
            current_row, current_col = self.get_state()
            new_row, new_col = current_row + row_change, current_col + col_change

            return [new_row, new_col]
        
        return None
    
    """Moves robot state left by 1 position."""
    def move_left(self):

        if "left" in self.possible_moves:
            row_change, col_change = self.possible_moves["left"]
            current_row, current_col = self.get_state()
            new_row, new_col = current_row + row_change, current_col + col_change

            return [new_row, new_col]
        
        return None

    """Moves robot state right by 1 position."""
    def move_right(self):

        if "right" in self.possible_moves:
            row_change, col_change = self.possible_moves["right"]
            current_row, current_col = self.get_state()
            new_row, new_col = current_row + row_change, current_col + col_change

            return [new_row, new_col]
        
        return None

    """Moves robot state up by 1 position and left by 1 position."""
    def move_up_left(self):

        if "up_left" in self.possible_moves:
            row_change, col_change = self.possible_moves["up_left"]
            current_row, current_col = self.get_state()
            new_row, new_col = current_row + row_change, current_col + col_change

            return [new_row, new_col]
        
        return None

    """Moves robot state up by 1 position and right by 1 position."""
    def move_up_right(self):

        if "up_right" in self.possible_moves:
            row_change, col_change = self.possible_moves["up_right"]
            current_row, current_col = self.get_state()
            new_row, new_col = current_row + row_change, current_col + col_change

            return [new_row, new_col]
        
        return None

    """Moves robot state down by 1 position and left by 1 position."""
    def move_down_left(self):

        if "down_left" in self.possible_moves:
            row_change, col_change = self.possible_moves["down_left"]
            current_row, current_col = self.get_state()
            new_row, new_col = current_row + row_change, current_col + col_change

            return [new_row, new_col]
        
        return None

    """Moves robot state down by 1 position and right by 1 position."""
    def move_down_right(self):

        if "down_right" in self.possible_moves:
            row_change, col_change = self.possible_moves["down_right"]
            current_row, current_col = self.get_state()
            new_row, new_col = current_row + row_change, current_col + col_change

            return [new_row, new_col]
        
        return None

"""This class handles all the actions related to finding the optimal path to the goal state using Breadth First Search."""
class SolverBFS:
    def __init__(self, initial_state, goal_state=np.array([1,1])):

        self.point_robot = PointRobot()
        #self.possible_moves = self.point_robot.get_possible_moves()
        #print(self.possible_moves)
        self.initial_state = initial_state
        self.goal_state = goal_state

        self.visited_states = {}
        self.queue = deque([self.initial_state])

        self.node_counter = 1

        self.node_path = []
        self.node_info = []
        self.nodes_explored = []



    def get_goal_position(self, valid_goal=False):
        while valid_goal != True:

            goal_point = input("Enter goal position as x, y: ")
            goal_point = list(map(int, goal_point.split(", ")))
            valid_goal = self.check_goal_position(goal_point)
            

    """Checks to see if a goal point is valid."""
    def check_goal_position(self, goal_pt):
        if 0 <= goal_pt[0] < self.point_robot.map.map_width and 0 <= goal_pt[1] < self.point_robot.map.map_height: # Checks to see if start point is within bounds of map
            if self.point_robot.map.grid[goal_pt[0], goal_pt[1]] != -1:

                self.goal_state = np.array(goal_pt)
                print("Valid goal point!")
                return True
            else:
                print("Goal point located in obstacle! Try again.")
                return False
        else:
            print("Goal point out of map boundary! Try again.")
            return False


    """Performs the BFS algorithm with backtracking to find the optimal path from initial state to goal state."""
    def search_path_bfs(self):
        
        #self.possible_moves = self.point_robot.get_possible_moves()

        while self.queue:
            current_state = self.queue.popleft()
            

            self.nodes_explored.append(current_state.node_state_i)
            self.node_info.append([current_state.node_index_i, current_state.parent_node_index_i, current_state.node_state_i])

            if np.array_equal(current_state.node_state_i, self.goal_state):
                
                print("Found goal state!")
                self.generate_path(current_state)
                self.write_files()

                return self.node_path
            
            self.visited_states[tuple(current_state.node_state_i)] = current_state

            for move in current_state.possible_moves.keys():
                
                new_robot_state = None

                if move == "up":
                    new_robot_state = current_state.move_up()
                elif move == "down":
                    new_robot_state = current_state.move_down()
                elif move == "left":
                    new_robot_state = current_state.move_left()
                elif move == "right":
                    new_robot_state = current_state.move_right()
                elif move == "up_left":
                    new_robot_state = current_state.move_up_left()
                elif move == "up_right":
                    new_robot_state = current_state.move_up_right()
                elif move == "down_left":
                    new_robot_state = current_state.move_down_left()
                elif move == "down_right":
                    new_robot_state = current_state.move_down_right()
                
                if new_robot_state is not None:
                    new_state = PointRobot(new_robot_state, self.node_counter, current_state.node_index_i)

                    if tuple(new_state.node_state_i) not in self.visited_states:
                        new_state = PointRobot(new_robot_state, self.node_counter, current_state.node_index_i)
                        self.node_counter += 1
                        self.queue.append(new_state) 
                        self.visited_states[tuple(new_state.node_state_i)] = new_state

        print('Exited loop, this configuration is not solvable!')

    """Generates a path using backtracking fromt the goal state back to the initial state."""
    def generate_path(self, goal_state):

        current_state = goal_state
        path = []

        while current_state is not None:

            path.append(current_state)

            if current_state.node_index_i == 0:
                break

            current_state = self.get_parent_state(current_state)
            
        self.node_path = path[::-1] # Sets the node path equal to the reverse of path

    """Determines the parent node state to move through the graph."""
    def get_parent_state(self, state):

        if state.parent_node_index_i is None:
            return None
        
        for s in self.visited_states.values():
            if s.node_index_i == state.parent_node_index_i:
                return s
        
        return None
    
    """Writes the 3 output files from the generated lists."""
    def write_files(self):
        with open("Nodes.txt", "w") as nodes_file:
            for state in self.nodes_explored:
                nodes_file.write(" ".join(map(str, state)) + "\n")
        
        with open("NodesInfo.txt", "w") as nodes_info_file:
            for node_info in self.node_info:
                node_state = " ".join(map(str, node_info[2]))
                nodes_info_file.write(f"{node_info[0]}\t{node_info[1]}\t{node_state}\n")

        with open("nodePath.txt", "w") as node_path_file:
            for state in self.node_path:
                node_path_file.write(" ".join(map(str, state.node_state_i)) + "\n")
    



        

    



"""Creating Code to Run:"""
#map_test = Map("map.txt", 180, 50)

#map_test.read_obstacles()

#map_utils = MapUtils()
#map_test.map_utils.write_obstacle_file(map_test.grid)



robot = PointRobot()
robot_s = SolverBFS(robot)

robot.map.read_obstacles()
robot.map.map_utils.write_obstacle_file(robot.map.grid)
robot.map.map_utils.write_flipped_obstacle_file(robot.map.grid)



robot.get_start_position()
robot_s.get_goal_position()

#robot.get_possible_moves()
robot_s.search_path_bfs()

#robot.get_state()









