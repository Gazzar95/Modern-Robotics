"""
A* search steps:
1)go through each node 
2)estimate tot cost 
3)add those values back to OPEN
4)rank and start at lowest value node again
5)go back to checking all nodes


"""


def Astar(start, finish, nodes_dic, edge_dig):
    OPEN = []
    CLOSED = set()

    past_cost = {}
    for node in nodes_dic:
        past_cost[node] = float('inf')
    past_cost[start] = 0

    heuristic_cost_start = nodes_dic[start][2]
    tot_cost_start = heuristic_cost_start + past_cost[start]

    OPEN.append(tot_cost_start, past_cost[start], start)

    while len(OPEN) > 0:
        OPEN.sort()

        current = OPEN(0)

        current_tot_cost = current(0)
        current_past_cost = current(1)
        current_ID = current(2)

        OPEN.pop(0)

        CLOSED.add(current)

        if current == finish:
            print('done!')
            # then some other loop to get you the path

        # <=========INCORRECT didn't use get()
        nbr_list = edge_dig.get(current, [])

        for nbr_info in nbr_list:
            nbr = nbr_info(0)

            if nbr(0) in CLOSED:
                continue

            nbr_cost = nbr(1)
            tot_to_nbr_cost = nbr_cost+past_cost[current]
            if tot_to_nbr_cost < past_cost[nbr]:

                nbr_heuristic_cost = edge_dig[nbr][2]
                tot_nbr_cost = tot_to_nbr_cost + nbr_heuristic_cost

                OPEN.append(tot_nbr_cost, tot_to_nbr_cost, nbr)
