import os
import numpy as np
from typing import Tuple

# Get nodes and edges from csv files
nodes = np.loadtxt('Configs/Assignment-3/nodes.csv', delimiter=',')
edges = np.loadtxt('Configs/Assignment-3/edges.csv', delimiter=',')


def getAdjNodes(edges: np.array, node: int) -> Tuple[list, list]:
    """
    Function to find adjacent nodes and edge costs for a given node

    edges: numpy array: edges in the graph
    node: int: current node in the graph

    Returns: adj_nodes: list: adjacent nodes, costs: list: edge costs
    """

    adj_nodes = []
    costs = []
    
    # Find adjacent nodes and costs
    for edge in edges:
        if edge[0] == node:
            adj_nodes.append(int(edge[1]))
            costs.append(edge[2])
        elif edge[1] == node:
            adj_nodes.append(int(edge[0]))
            costs.append(edge[2])

    return adj_nodes, costs

def A_star(nodes: np.array, edges: np.array) -> list:
    """
    Function to find shortest path using A* algorithm

    nodes: numpy array: nodes in the graph
    edges: numpy array: edges in the graph

    Returns: route: list: shortest path
    """

    open_list = []
    closed_list = []

    # Start and goal nodes
    start_idx = 1
    current_node = start_idx
    end_idx = nodes.shape[0]
    open_list.append(start_idx)

    # Set up parent nodes, past cost, and heuristic cost
    parent_nodes = np.inf*np.ones((end_idx))
    parent_nodes[start_idx-1] = 0
    past_cost = np.inf*np.ones((end_idx))
    past_cost[start_idx-1] = 0
    heuristic_cost = nodes[:,3]

    # Expand open nodes
    while open_list != []:
        if current_node == end_idx:
            break

        # Find adjacent nodes and costs
        adj_nodes, costs = getAdjNodes(edges,current_node)

        # Iterate through adjacent nodes
        for i in range(len(adj_nodes)):
            if (adj_nodes[i] not in open_list) and (adj_nodes[i] not in closed_list):
                    open_list.append(adj_nodes[i])

            # Add cost to current node
            parent_cost = past_cost[current_node-1]
            current_cost = parent_cost + costs[i]

            if current_cost < past_cost[adj_nodes[i]-1]:
                parent_nodes[adj_nodes[i]-1] = current_node
                past_cost[adj_nodes[i]-1] = current_cost

        # Move current node to closed list
        open_list.remove(current_node)
        closed_list.append(current_node)

        total_cost = past_cost + heuristic_cost

        # Find lowest cost from open nodes
        open_costs = []
        for j in range(len(open_list)):
            open_costs.append(total_cost[open_list[j]-1])

        # Choose new current node
        current_node = open_list[np.argmin(open_costs)]

    if current_node != end_idx:
        return []

    # Backtrack to find the route
    route = []
    route_current = current_node
    route.append(route_current)
    while route_current != start_idx:
        route.insert(0, int(parent_nodes[route_current-1]))
        route_current = int(parent_nodes[route_current-1])
        
    return route

# Apply A* algorithm to find the shortest path
path = A_star(nodes,edges)

# Save the data to CSV file
os.makedirs('Configs/Assignment-3/', exist_ok=True)
filepath = os.path.join('Configs/Assignment-3/', 'path.csv')
np.savetxt(filepath, np.array(path).reshape(1, -1), delimiter=',', fmt='%d')