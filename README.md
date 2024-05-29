# Planar Serial Manipulator Path Planning
Path planning for a serial manipulator using Rapidly-Exploring-Random Tree (RRT) and  Probabilistic Roadmaps (PRM)

This project implements path planning algorithms for an n-link planar serial manipulator in a 2D workspace with obstacles utilzing MATLAB. The goal is to find a collision-free path from a given initial configuration to a goal configuration using two different algorithms: Probabilistic Road Map (PRM) and Rapidly-Exploring Random Tree (RRT).

## Project Structure

- `t1_main.m`: The main script that sets up the workspace, obstacles, and manipulator parameters, and calls the path planning functions.
- `manipulator_RRT.m`: Implementation of the RRT algorithm for path planning.
- `manipulator_PRM.m`: Implementation of the PRM algorithm for path planning.
- `dijkstra.m`: Implementation of Dijkstra's shortest path algorithm for finding the optimal path in the constructed graphs.

## Dependencies

- MATLAB

## Usage

1. Open MATLAB and navigate to the project directory.
2. Open the `t1_main.m` script.
3. Modify the workspace dimensions, obstacles, number of links, link lengths, initial configuration, and goal configuration as desired.
4. Run the `t1_main.m` script to execute the path planning algorithms.
5. The script will generate plots of the workspace, obstacles, initial and goal configurations, and the planned paths for both RRT and PRM algorithms.
6. The generated paths will be saved as video files named `rrt_movie.mp4` and `prm_movie.mp4` in the project directory.

## Results
The project successfully plans collision-free paths for the planar serial manipulator using both RRT and PRM algorithms. The planned paths are visualized in the generated plots and saved as video files for further analysis.

## Limitations and Future Work

- The current implementation assumes fully rotary joints and does not consider self-collisions between the links.
- The collision checking is performed using a simple point-in-polygon test and may not be efficient for complex obstacle shapes.
- Future work could include extending the algorithms to higher-dimensional configuration spaces and considering additional constraints such as joint limits and self-collisions.

