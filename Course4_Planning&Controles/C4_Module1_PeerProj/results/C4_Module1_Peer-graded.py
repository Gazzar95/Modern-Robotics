# %%
# Course 4, Module 1: Peer-graded Assignment
# Inputs: nodes.csv, edges.csv
# Outputs: path.csv

import csv
import math

# Load node positions from nodes.csv
def load_nodes(filename):
    node_positions = {}
    with open(filename, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if not row or row[0].startswith('#'):
                continue  # skip empty lines or comments
            node_id = int(row[0])
            x = float(row[1])
            y = float(row[2])
            node_positions[node_id] = (x, y)
    return node_positions

# Load edges and their costs from edges.csv (bidirectional)
def load_edges(filename):
    edges = {}
    with open(filename, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if not row or row[0].startswith('#'):
                continue  # skip empty lines or comments
            src = int(row[0])
            dst = int(row[1])
            cost = float(row[2])

            # Add forward edge
            if src not in edges:
                edges[src] = []
            edges[src].append((dst, cost))

            # Add reverse edge
            if dst not in edges:
                edges[dst] = []
            edges[dst].append((src, cost))
    return edges

# Heuristic cost-to-go (Euclidean distance)
def heuristic_cost_to_go(n, goal, node_positions):
    x1, y1 = node_positions[n]
    x2, y2 = node_positions[goal]
    return math.hypot(x2 - x1, y2 - y1)

# A* Algorithm using real edge costs
def AstarSearch(start, goal, node_positions, edges):
    OPEN = [(heuristic_cost_to_go(start, goal, node_positions), 0, start)]
    CLOSED = set()

    past_cost = {node: float('inf') for node in node_positions}
    past_cost[start] = 0

    parent = {}

    while OPEN:
        OPEN.sort()
        print("OPEN:", OPEN)  # Debug: show current OPEN list
        _, cost_to_current, current = OPEN.pop(0)
        print(f"Exploring node {current} with past cost {cost_to_current}")

        if current == goal:
            print("Goal reached!")
            path = [current]
            while current in parent:
                current = parent[current]
                path.append(current)
            return list(reversed(path))

        CLOSED.add(current)

        for nbr, edge_cost in edges.get(current, []):
            if nbr in CLOSED:
                continue

            new_cost = past_cost[current] + edge_cost
            print(f"  Checking neighbor {nbr}, edge cost {edge_cost}, new cost {new_cost}")

            if new_cost < past_cost[nbr]:
                past_cost[nbr] = new_cost
                parent[nbr] = current
                est_total_cost = new_cost + heuristic_cost_to_go(nbr, goal, node_positions)
                print(f"  Updating {nbr}: total estimate {est_total_cost}")
                OPEN.append((est_total_cost, new_cost, nbr))

    print("Search exhausted. No path found.")
    return None

# Write path to path.csv as a single row
def write_path(path, filename):
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(path)

# Main script
if __name__ == "__main__":
    node_positions = load_nodes("nodes.csv")
    edges = load_edges("edges.csv")
    start_node = 1
    goal_node = max(node_positions.keys())
    print(f"Start node: {start_node}, Goal node: {goal_node}")

    path = AstarSearch(start_node, goal_node, node_positions, edges)

    if path:
        print("Path found:", path)
        write_path(path, "path.csv")
    else:
        print("No path found.")

# %%
