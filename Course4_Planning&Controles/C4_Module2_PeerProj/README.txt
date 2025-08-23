README - Motion Planning with RRT
Author: Omar El Gazzar
Planner: Rapidly-Exploring Random Tree (RRT)
Overview

This solution implements a Rapidly-Exploring Random Tree (RRT) algorithm to solve a 2D point robot motion planning problem in a workspace with circular obstacles. The planner is designed to generate a feasible collision-free path from a given start position to a goal position.
Algorithm Choice

I chose RRT due to its efficiency in exploring large, high-dimensional spaces and its suitability for problems where finding a feasible path is more important than finding the optimal one. RRT was implemented from scratch using modular Python code, without external planning libraries.
Key Implementation Details

    Workspace: 2D bounded square from (-0.5, -0.5) to (0.5, 0.5)

    Sampling Strategy: 10% of samples are biased toward the goal to improve convergence

    Collision Checking: A custom function checks for line segment intersections with circular obstacles (inflated slightly for safety)

    Step Size: 0.1 units

    Goal Radius: 0.1 units to allow proximity-based termination

    Max Tree Size: 1000 nodes

Files

    planner.py: Main script containing the RRT implementation

    obstacles.csv: List of circular obstacles (x,y,r)

    nodes.csv: Generated node list for CoppeliaSim visualization (ID,x,y)

    edges.csv: Generated edge list with cost (ID1,ID2,cost)

    path.csv: Final solution path as a list of node IDs

    results/: Directory where all output CSVs are written

Notes

    The planner starts with node index 1 to comply with CoppeliaSim’s graph input format.

    Obstacle radii are slightly reduced (r - 0.08) to avoid false positives in collision checking due to edge grazing.

    No additional input files are required beyond obstacles.csv.