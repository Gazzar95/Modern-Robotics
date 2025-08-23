# %%
# Create a beginner-friendly, modular RRT implementation with proper file outputs and structure.
import csv
import math
import random
import os

# Define workspace boundaries
X_LIMITS = (-0.5, 0.5)
Y_LIMITS = (-0.5, 0.5)
STEP_SIZE = 0.1
MAX_TREE_SIZE = 1000
GOAL_SAMPLE_RATE = 0.1
GOAL_RADIUS = 0.1

# Define start and goal
x_start = (-0.5, -0.5)
x_goal = (0.5, 0.5)

# Load obstacle list from CSV file


def load_obstacles(filename):
    obstacles = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            if not row or row[0].startswith('#'):
                continue
            x, y, r = map(float, row)
            obstacles.append((x, y, r-0.08))
    return obstacles

# Collision checking helpers


def line_intersects_circle(p1, p2, circle):
    cx, cy, r = circle
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    fx = p1[0] - cx
    fy = p1[1] - cy

    a = dx**2 + dy**2
    b = 2 * (fx * dx + fy * dy)
    c = fx**2 + fy**2 - r**2

    discriminant = b**2 - 4 * a * c
    if discriminant < 0:
        return False

    discriminant = math.sqrt(discriminant)
    t1 = (-b - discriminant) / (2 * a)
    t2 = (-b + discriminant) / (2 * a)

    return (0 <= t1 <= 1) or (0 <= t2 <= 1)


def is_edge_collision(p1, p2, obstacles):
    for circle in obstacles:
        if line_intersects_circle(p1, p2, circle):
            return True
    return False

# Node data structure


class Node:
    def __init__(self, pos, parent=None, index=0):
        self.pos = pos        # A 2D point (x, y)
        self.parent = parent  # Index of parent node
        self.index = index    # Index of this node in the node list

# RRT implementation


def rrt(obstacles):
    nodes = [Node(x_start, parent=None, index=1)]
    edges = []

    for i in range(1, MAX_TREE_SIZE):
        # Sample
        if random.random() < GOAL_SAMPLE_RATE:
            x_samp = x_goal
        else:
            x_samp = (
                random.uniform(*X_LIMITS),
                random.uniform(*Y_LIMITS)
            )

        # Find nearest node
        min_dist = float('inf')
        x_nearest = None

        for node in nodes:
            dx = node.pos[0] - x_samp[0]
            dy = node.pos[1] - x_samp[1]
            dist = math.hypot(dx, dy)

            if dist < min_dist:
                min_dist = dist
                x_nearest = node

        # Steer
        theta = math.atan2(x_samp[1] - x_nearest.pos[1],
                           x_samp[0] - x_nearest.pos[0])
        x_new = (
            x_nearest.pos[0] + STEP_SIZE * math.cos(theta),
            x_nearest.pos[1] + STEP_SIZE * math.sin(theta)
        )

        if not (X_LIMITS[0] <= x_new[0] <= X_LIMITS[1] and Y_LIMITS[0] <= x_new[1] <= Y_LIMITS[1]):
            continue

        if is_edge_collision(x_nearest.pos, x_new, obstacles):
            continue

        new_node = Node(x_new, parent=x_nearest.index, index=len(nodes)+1)
        nodes.append(new_node)
        edges.append((x_nearest.index, new_node.index))

        if math.hypot(x_new[0] - x_goal[0], x_new[1] - x_goal[1]) <= GOAL_RADIUS:
            print(
                f"Goal reached at iteration {i} with node index {new_node.index}")
            goal_node = Node(x_goal, parent=new_node.index, index=len(nodes)+1)
            nodes.append(goal_node)
            edges.append((new_node.index, goal_node.index))
            return nodes, edges, goal_node

    return nodes, edges, None

# Path backtracking


def extract_path(nodes, goal_node):
    path = []
    current = goal_node
    while current is not None:
        path.append(current)
        parent_index = current.parent
        current = next((n for n in nodes if n.index == parent_index), None)
    return path[::-1]

# Write output CSVs


def write_outputs(nodes, edges, path, out_dir='results'):
    print(f"Writing output to: {os.path.abspath(out_dir)}")
    os.makedirs(out_dir, exist_ok=True)

    with open(os.path.join(out_dir, 'nodes.csv'), 'w', newline='') as f_nodes:
        writer = csv.writer(f_nodes)
        writer.writerow(["# ID", "x", "y"])  # Header line
        for node in nodes:
            writer.writerow(
                [node.index, f"{node.pos[0]:.4f}", f"{node.pos[1]:.4f}"])

    with open(os.path.join(out_dir, 'edges.csv'), 'w', newline='') as f_edges:
        writer = csv.writer(f_edges)
        writer.writerow(["# ID1", "ID2", "cost"])  # Optional header
        for parent_idx, child_idx in edges:
            from_node = next(n for n in nodes if n.index == parent_idx)
            to_node = next(n for n in nodes if n.index == child_idx)

            cost = math.hypot(to_node.pos[0] - from_node.pos[0],
                              to_node.pos[1] - from_node.pos[1])
            writer.writerow([parent_idx, child_idx, f"{cost:.4f}"])

    with open(os.path.join(out_dir, 'path.csv'), 'w', newline='') as f_path:
        writer = csv.writer(f_path)
        writer.writerow(
            ["# All lines beginning with a # are treated as a comment and ignored."])
        writer.writerow(
            ["# Below is a solution path represented as a sequence of nodes of the graph."])
        path_ids = [node.index for node in path]
        writer.writerow(path_ids)


# MAIN EXECUTION
obstacles = load_obstacles('obstacles.csv')

nodes, edges, goal_node = rrt(obstacles)
if goal_node:
    print("Path found!")
    path = extract_path(nodes, goal_node)
else:
    print("Path NOT found.")
    path = []


print("Saving files to:", os.path.abspath('results'))
write_outputs(nodes, edges, path)
print("Outputs written successfully.")

# %%
