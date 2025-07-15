Sampling-Based Planning Project

This repository contains my implementation of a sampling-based motion planner (RRT or PRM) for a point robot navigating a cluttered 2D environment, as outlined in the Modern Robotics Course 4 assignment from Northwestern University
Robotics Proceedings+5hades.mech.northwestern.edu+5hades.mech.northwestern.edu+5
.
🚀 Goal

To reinforce my understanding of motion planning and Python programming basics by:

    Implementing either an RRT or PRM from scratch in Python.

    Practicing core Python features (functions, control flow, file I/O).

    Validating algorithm correctness through data output and CoppeliaSim visualizations.

Project Structure

sampling_planner/
├── obstacles.csv      # Input obstacle definitions (x, y, radius)
├── nodes.csv          # Generated roadmap/tree nodes
├── edges.csv          # Connections between nodes
├── path.csv           # Computed path from start to goal
├── planner.py         # Main sampling-based planner script (RRT/PRM)
└── README.md          # This documentation

How It Works

    Input:
    Uses obstacles.csv for obstacle locations and radii.

    Sampling in C-space:
    Samples random (x, y) in the square [-0.5, 0.5] × [-0.5, 0.5].

    Planner Options:

        RRT: Builds a tree using the nearest-node + straight-line local planner.

        PRM: Constructs a sparse roadmap by sampling evenly and connecting k-nearest neighbors if free.

    Collision Checking:
    Validates straight-line segments against obstacles.

    Output Files:

        nodes.csv: Node coordinates.

        edges.csv: Pairs of node indices representing edges.

        path.csv: Sequence of node indices forming a connecting path (if found).

    Visualization:
    Use CoppeliaSim scene to visualize, or load data in Python to plot the roadmap and planned path.

Running the Planner

python planner.py --mode rrt     # Run as RRT
python planner.py --mode prm     # Run as PRM

Outputs nodes.csv, edges.csv, and path.csv in the working directory.
Learning Outcomes

    Hands-on experience with sampling-based algorithms: RRT/PRM
    hades.mech.northwestern.edu+3hades.mech.northwestern.edu+3orensalzman.com+3
    orensalzman.com
    .

    Applied collision detection and nearest-neighbor logic.

    Reinforced fundamentals of Python: CSV I/O, data structures, control flow.

    Visual verification via CoppeliaSim snapshots as required for grading.

Next Steps

    Add smoothing to the planned path.

    Optimize sampling strategy or connectivity rules.

    Extend to higher dimensions or kinodynamic planning.