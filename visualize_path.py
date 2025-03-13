"""This code will visualize the point robot searching, and finding the optimal path from the start point to the goal ponint."""

import pygame
import numpy as np

pygame.init()

# Constants
MAP_WIDTH, MAP_HEIGHT = 1800, 500
BACKGROUND = (0, 0, 0)
ROBOT_COLOR = (0, 255, 0)
FINAL_PATH_COLOR = (255, 255, 0)
OBSTACLE_COLOR = (255, 0, 0)

CELL_SIZE = 10  # Size of each cell in the grid


def read_map_data(filename):
    obstacles = []

    with open(filename, "r") as f:
        for line in f:
            shapes = line.strip().split(", ")
            obstacle_type = shapes[0]

            if obstacle_type == "rect":
                x, y, width, height = map(int, shapes[1:5])
                color = tuple(map(int, shapes[5:8]))
                obstacles.append(("rect", x, y, width, height, color))
            
            if obstacle_type == "circle":
                x, y, radius = map(int, shapes[1:4])
                color = tuple(map(int, shapes[4:7]))
                obstacles.append(("circle", x, y, radius, color))

            if obstacle_type == "parallel":
                x1, y1, x2, y2, x3, y3, x4, y4 = map(int, shapes[1:9])
                color = tuple(map(int, shapes[9:12]))
                obstacles.append(("parallel", x1, y1, x2, y2, x3, y3, x4, y4, color))
    
    return obstacles

def show_map_data(screen, obstacles):
    for obstacle in obstacles:
        if obstacle[0] == "rect":
            _, x, y, width, height, color = obstacle
            pygame.draw.rect(screen, color, (x, y, width, height))
        if obstacle[0] == "circle":
            _, x, y, radius, color = obstacle
            pygame.draw.circle(screen, color, (x, y), radius)
        if obstacle[0] == "parallel":
            _, x1, y1, x2, y2, x3, y3, x4, y4, color = obstacle
            pygame.draw.polygon(screen, color, [(x1, y1), (x2, y2), (x3, y3), (x4, y4)])

            
def read_output_data(filename):
    grid = []
    with open(filename, "r") as f:
        for line in f:
            row = list(map(int, line.strip().split()))
            grid.append(row)
    return np.array(grid)

"""Visualize the grid and the robot in a separate window."""
def show_obstacle_space(grid, screen):
    
    # Draw the grid and obstacles
    for y in range(grid.shape[0]):
        for x in range(grid.shape[1]):
            if grid[y, x] == -1:  # obstacle
                pygame.draw.rect(screen, OBSTACLE_COLOR, (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE))

"""Reads search_path.txt file for extract robot search path.
Will need to update this once I make more progress on the 
BFS implementation.
"""
def read_search_path_data(filename):
    search_path = []
    with open(filename, "r") as f:
        for line in f:
            row = list(map(int, line.strip().split()))
            search_path.append(row)
    #print(search_path)
    return np.array(search_path)


obstacles = read_map_data("map.txt")
grid = read_output_data("obstacle_output_file.txt")

search_path = read_search_path_data("Nodes.txt")
optimal_path = read_search_path_data("nodePath.txt")
#search_path = read_search_path_data("search_path.txt")

goal = optimal_path[-1]
goal_x, goal_y = goal[1], abs((MAP_HEIGHT // 10 -1 ) - goal[0]) 
start = optimal_path[0]
start_x, start_y = start[1], abs((MAP_HEIGHT // 10 -1) - start[0])


screen = pygame.display.set_mode((MAP_WIDTH, MAP_HEIGHT))
pygame.display.set_caption("ENPM661 MAP")


run = True
while run:

    pygame.time.delay(30)
    screen.fill(BACKGROUND)

    #show_obstacle_space(grid, screen)    # Show obstacle space with clearance
    show_map_data(screen, obstacles)    # SHow map data of obstacles
    

    
    """
    The following code loops through the Nodes.txt 
    and displays the robot searching for the goal position.
    """
    
    for state in search_path:

        pygame.draw.rect(screen, (255, 0, 0), (goal_x * CELL_SIZE, goal_y * CELL_SIZE, CELL_SIZE, CELL_SIZE))
        pygame.draw.rect(screen, (0, 0, 255), (start_x * CELL_SIZE, start_y * CELL_SIZE, CELL_SIZE, CELL_SIZE))

        for event in pygame.event.get(): # Checking for mouse clicking EXIT button
            if event.type == pygame.QUIT:
                run = False

        #print(state)
        x = state[1]
        y = abs((MAP_HEIGHT // 10 - 1) - state[0])
        pygame.draw.rect(screen, ROBOT_COLOR, (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE))

        pygame.display.update()
        pygame.time.delay(1)

    """
    The following code loops through the nodePath.txt 
    and displays the optimal path to the goal.
    """
    for state in optimal_path:

        pygame.draw.rect(screen, (255, 0, 0), (goal_x * CELL_SIZE, goal_y * CELL_SIZE, CELL_SIZE, CELL_SIZE))
        pygame.draw.rect(screen, (0, 0, 255), (start_x * CELL_SIZE, start_y * CELL_SIZE, CELL_SIZE, CELL_SIZE))

        for event in pygame.event.get(): # Checking for mouse clicking EXIT button
            if event.type == pygame.QUIT:
                run = False

        #print(state)
        x = state[1]
        y = abs((MAP_HEIGHT // 10 - 1) - state[0])
        pygame.draw.rect(screen, FINAL_PATH_COLOR, (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE))

        pygame.display.update()
        pygame.time.delay(50)


    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False


    
    pygame.time.delay(5000)
    pygame.quit()
    
pygame.quit()