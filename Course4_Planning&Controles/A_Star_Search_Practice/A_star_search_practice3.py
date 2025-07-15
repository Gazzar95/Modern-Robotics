# nodes = {node: (x, y, hueristic)}

# edge_dic = {node: (nbr1, trvl_cost1), (nbr2, trvl_cost2),...(nbrn, trvl_costn)}


def AStarAlgorithm(start, goal, nodes_dic, edge_dic):

    OPEN = []

    CLOSE = set()

    past_cost = {}

    for node in nodes_dic:

        past_cost[node] = float('inf')

    past_cost[start] = 0

    start_hueristic_cost = nodes_dic[start][2]

    start_tot_cost = start_hueristic_cost + past_cost[start]

    OPEN.append(start_tot_cost, past_cost[start], start)

    while len(OPEN) > 0:

        OPEN.sort()

        current = OPEN[0]

        past_cost = current(1)

        current_ID = current(2)

        CLOSE.add(current)

        OPEN.pop(0)

        # need to understand this condition -----------------------

        if current_ID == goal:

            print('DONE!')

            path = [1, 3, 4]

            break

        # need to understand this condition -----------------------

        # <------- got this incorrect again! Need to use ".get(current, [])"
        nbr_list_all = edge_dic[current_ID]

        for nbr_list in nbr_list_all:

            nbr = nbr_list(0)

            trvl_cost = nbr_list(1)

            if nbr in CLOSE:

                continue

            nbr_past_cost = trvl_cost + past_cost

            if nbr_past_cost < past_cost[nbr]:

                nbr_heuristic = nodes_dic[nbr][2]

                nbr_tot_cost = nbr_past_cost + nbr_heuristic

                # <-------- should have used "nbr_past_cost" instead
                past_cost[nbr] = nbr_tot_cost

                OPEN.append(nbr_tot_cost, nbr_past_cost, nbr)

    return path
