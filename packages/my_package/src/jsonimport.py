import json
import math

class Node:
    def __init__(self):
        self.x = 0.0  # x-coordinate [m]
        self.y = 0.0  # y-coordinate [m]
        self.g = 0.0  # cost-to-come from the start to this node
        self.h = 0.0  # heuristic cost-to-go from this node to the goal
        self.f = 0.0  # total cost (cost-to-come g + heuristic cost-to-go h)
        self.parent_node_ID = 0  # ID of the node from which node you arrived at this node with the lowest cost-to-come

def calculate_distance(node_A, node_B):
    return math.sqrt((node_A.x - node_B.x) ** 2 + (node_A.y - node_B.y) ** 2)

if __name__ == '__main__':
    # Read the JSON config file
    with open('FInal_project/We-will-do-it/packages/my_package/config/params_maze_small.json') as file:
        configfile = json.load(file)

    # Default config values, will be overwritten if the config file contains the correct information
    resolution = 0.0
    pixels_per_node = 0.0
    node_resolution = 0.0
    max_nodes_x = 0
    max_nodes_y = 0
    n_nodes = 0

    if 'nodes' in configfile:
        n_nodes = len(configfile['nodes'])

    nodelist = [Node() for _ in range(n_nodes)]
    connections = [[] for _ in range(n_nodes)]
    entrance_node_id = 0
    finish_node_id = 0

    # Extract relevant information from the config file
    if 'World' in configfile and 'OccupancyGridMap' in configfile['World']:
        occupancy_grid_map = configfile['World']['OccupancyGridMap']
        max_nodes_x = occupancy_grid_map.get('max_nodes_x', 0)
        max_nodes_y = occupancy_grid_map.get('max_nodes_y', 0)
        resolution = occupancy_grid_map.get('resolution', 0.0)
        pixels_per_node = occupancy_grid_map.get('pixels_per_node', 0.0)
        node_resolution = resolution * pixels_per_node
        print("Resolution:", resolution)

    if 'nodes' in configfile and 'entrance' in configfile and 'finish' in configfile and 'connections' in configfile:
        nodes = configfile['nodes']
        entrance_node_id = configfile['entrance']
        finish_node_id = configfile['finish']

        for i_node in range(n_nodes):
            new_node = Node()
            new_node.x = node_resolution * (0.5 * max_nodes_x - (nodes[i_node][0] - 0.5))
            new_node.y = node_resolution * (0.5 * max_nodes_y - (nodes[i_node][1] - 0.5))
            new_node.g = float('inf')
            new_node.h = 0.0
            new_node.f = new_node.g + new_node.h
            nodelist[i_node] = new_node

            connections[i_node] = configfile['connections'][i_node]

        print("Entrance Node ID:", entrance_node_id)
        print("Exit Node ID:", finish_node_id)
        print("Connections:", connections[1][1])
