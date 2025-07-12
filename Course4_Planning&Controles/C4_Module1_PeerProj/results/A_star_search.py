# %%
"""
Course 4, module 1 - Motion Planning
Project - Writeup A* Algorithm

goal:
    input - nodes & edges csv files
        data in node: ID,x,y,heuristic-cost-to-go
        data in edge: ID1,ID2,cost
    output - path from start to finish
        data in path: node id sequence between start and finish
Procedure:
    1)translate node and edge .scv into dictionary in py
    2) create A* algorithm function
        a)create an OPEN list where each item
        b)Algorithm to check edge dic and review neighboring nodes to start node
            - Calc Tot cost = past cost + heuristic and order in OPEN list
        c) pop() the node and throw in CLOSED list
        d) repeat till we hit end node
"""

 

import csv

 

# Procedure 1 - import node & edge .csv files -----------------------------

 

# import nodes

 

def ImportNodes(filename):
    nodes_dic = {}
    
    with open(filename, 'r') as csv_nodes:
        csv_nodes_content = csv.reader(csv_nodes)
        
        for row in csv_nodes_content:
            
            if row[0].startswith('#'):
                continue

            nodeID = int(row[0])
            node_x = float(row[1])
            node_y = float(row[2])
            node_heuristic = float(row[3])
            nodes_dic[nodeID] = (node_x, node_y, node_heuristic)

    return nodes_dic

 

# import edges
def ImportEdge(filename):

    edge_dic = {}

    with open(filename, 'r') as csv_edge:
        read_edges = csv.reader(csv_edge)

        for row in read_edges:
           
            if row[0].startswith('#'):
                continue

            src = int(row[0])
            dst = int(row[1])
            cost = float(row[3])

            if src not in read_edges:
                edge_dic[src] = []
            edge_dic[src] = (dst, cost)

            if dst not in read_edges:
                edge_dic[dst] = []
            edge_dic[dst] = (src, cost)

    return edge_dic

 

# Procedure 2 - create A* algorithm function -----------------------------

 

def a_star(start, end, nodes_dic, edge_dic):

    OPEN = []
    CLOSED = set()

    past_cost = {}
    for node in nodes_dic:
        past_cost[node] = float('inf')
    past_cost[start] = 0

    start_tot = past_cost[start]+nodes_dic[start][3]

    OPEN.append((start_tot, past_cost[start], start))

    while len(OPEN)>0:

        OPEN.sort()
        tot_cost, past_cost, current = OPEN.pop()
        CLOSED.add(current)

        neighbor_list = edge_dic.get(current, [])

        for neightbor_info in neighbor_list:
            nbr = neightbor_info[0]
            cost = neightbor_info[1]
            
            
            if nbr in CLOSED
                continue

            