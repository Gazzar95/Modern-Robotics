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


def AstarSearch(start, goal, node_positions, edges):
    # === Step 1: Initialize the OPEN list ===
    start_position = node_positions[start]
    goal_position = node_positions[goal]

    # Cost so far to reach the start node is zero
    start_past_cost = 0

    # Heuristic estimate from start to goal
    start_heuristic = heuristic_cost_to_go(start, goal, node_positions)

    # Total estimated cost (f = g + h)
    start_total_estimate = start_past_cost + start_heuristic

    # Add the start node to OPEN list
    OPEN = []
    OPEN.append((start_total_estimate, start_past_cost, start))

    # === Step 2: Initialize the CLOSED set ===
    CLOSED = set()

    # === Step 3: Initialize past_cost for each node ===
    past_cost = {}
    for node in node_positions:
        # Assume all nodes are unreachable initially
        past_cost[node] = float('inf')

    # The cost to reach the start node is 0
    past_cost[start] = 0

    # === Step 4: Initialize the parent map ===
    parent = {}

    # === Step 5: Main A* loop ===
    while len(OPEN) > 0:
        # Sort OPEN by estimated total cost (f = g + h)
        OPEN.sort()

        # Get the node with the lowest total estimated cost
        best_option = OPEN[0]
        est_total_cost = best_option[0]
        cost_to_current = best_option[1]
        current = best_option[2]

        # Remove it from the OPEN list
        OPEN.pop(0)

        print("OPEN:", OPEN)
        print(f"Exploring node {current} with past cost {cost_to_current}")

        # === Step 6: Goal check ===
        if current == goal:
            print("Goal reached!")
            path = []
            path.append(current)

            # Reconstruct the path using the parent map
            while current in parent:
                current = parent[current]
                path.append(current)

            # Reverse the path so it goes from start → goal
            path = list(reversed(path))
            return path

        # === Step 7: Add current to CLOSED ===
        CLOSED.add(current)

        # === Step 8: Get neighbors ===
        neighbor_list = edges.get(current, [])

        # === Step 9: Check each neighbor ===
        for neighbor_info in neighbor_list:
            nbr = neighbor_info[0]        # Neighbor node ID
            edge_cost = neighbor_info[1]  # Cost from current to neighbor

            # Skip neighbor if we've already visited it
            if nbr in CLOSED:
                continue

            # Calculate the total cost to reach the neighbor
            past_cost_to_current = past_cost[current]
            cost_to_nbr = past_cost_to_current + edge_cost

            print(
                f"  Checking neighbor {nbr}, edge cost {edge_cost}, new cost {cost_to_nbr}")

            # If this path to neighbor is better than what we had before...
            if cost_to_nbr < past_cost[nbr]:
                # Update past_cost and parent
                past_cost[nbr] = cost_to_nbr
                parent[nbr] = current

                # Estimate remaining cost to goal (heuristic)
                heuristic_to_goal = heuristic_cost_to_go(
                    nbr, goal, node_positions)

                # Total estimated cost (f = g + h)
                est_total = cost_to_nbr + heuristic_to_goal

                print(f"  Updating {nbr}: total estimate {est_total}")

                # Add neighbor to OPEN for future exploration
                OPEN.append((est_total, cost_to_nbr, nbr))

    # === Step 10: No path found ===
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
