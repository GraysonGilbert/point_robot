This is my submission for Project 2. All files necessary for the project are located on my GitHub linked below. 
I have also sent collaboration invites to koustubh@umd.edu and rmonfare@umd.edu 

GitHub Repo: https://github.com/GraysonGilbert/point_robot

The submission contains the main program, as well as supplemental files. The main program is BFS_grayson_gilbert.py, 
and it uses numpy and deque from collections. To run this code simply press run, and the user will be prompted to 
input coordinates for the start and goal points. Be careful to input the coordinates correctly in a ‘x, y’ format. 
If a valid point is entered you will be notified. If an invalid point is entered, you will be notified and prompted 
to try again. A known good start point is 5, 7 and a known good goal point is 30, 85. Once the start and goal states 
are entered, the BFS algorithm will search for an optimal path, and print a message to the terminal when said path is 
found. An important note for the program to run, is that the map.txt file is needed. This map.txt file includes the 
locations of all the obstacles for this map. It is read in by the program and used to populate the grid of the map.

The visualization portion of the project is run in a separate program, titled visualize_path.py.  This uses 
numpy and pygame. After the main program has completed, it will save off a few different files. These files are 
obstacle_output_file.txt, nodePath.txt, Nodes.txt, and NodesInfo.txt. The last 3 files are leveraged from project 1 
to aid in visualizing the search algorithm and optimal path. Once the program is run, a pygame window will pop up and 
show the BFS algorithm in action. The start point is a blue square and the goal point is a red square. Once the goal is
found the optimal path is shown as a yellow path.
