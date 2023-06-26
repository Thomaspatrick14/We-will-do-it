import math

NODE_ID_NOT_SET = -1

class Node:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.g = float('inf')
        self.h = 0.0
        self.f = 0.0

def calculate_distance(node_A, node_B):
    return math.sqrt((node_A.x - node_B.x) ** 2 + (node_A.y - node_B.y) ** 2)

class AstarPlanner:
   def __init__(self, data):
        # Default config values
        self.resolution = 0.0
        self.pixels_per_node = 0.0
        self.node_resolution = 0.0
        self.max_nodes_x = 0
        self.max_nodes_y = 0
        self.n_nodes = 0

        self.nodelist = []
        self.connections = []
        self.entrance_node_id = 0
        self.finish_node_id = 0

        # Extract relevant information from the data
        resolution, pixels_per_node, max_nodes_x, max_nodes_y, nodes, connections, entrance_node_id, finish_node_id = extract_data_from_dict(data)

        self.max_nodes_x = max_nodes_x
        self.max_nodes_y = max_nodes_y
        self.resolution = resolution
        self.pixels_per_node = pixels_per_node
        self.node_resolution = self.resolution * self.pixels_per_node

        self.n_nodes = len(nodes)
        self.nodelist = [Node() for _ in range(self.n_nodes)]
        self.connections = [[] for _ in range(self.n_nodes)]

        self.entrance_node_id = entrance_node_id
        self.finish_node_id = finish_node_id

        for i_node in range(self.n_nodes):
            new_node = Node()
            new_node.x = self.node_resolution * (0.5 * self.max_nodes_x - (nodes[i_node][0] - 0.5))
            new_node.y = self.node_resolution * (0.5 * self.max_nodes_y - (nodes[i_node][1] - 0.5))
            new_node.g = float('inf')
            new_node.h = 0.0
            new_node.f = new_node.g + new_node.h
            self.nodelist[i_node] = new_node

            self.connections[i_node] = connections[i_node]

        print("Entrance Node ID:", self.entrance_node_id)
        print("Exit Node ID:", self.finish_node_id)
        print("Connections:", self.connections[1][1])


def extract_data_from_dict(data):
    world = data['World']
    occupancy_grid_map = world['OccupancyGridMap']
    resolution = occupancy_grid_map['resolution']
    pixels_per_node = occupancy_grid_map['pixels_per_node']
    max_nodes_x = occupancy_grid_map['max_nodes_x']
    max_nodes_y = occupancy_grid_map['max_nodes_y']
    
    nodes = data['nodes']
    connections = data['connections']
    entrance_node_id = data['entrance']
    finish_node_id = data['finish']
    
    return resolution, pixels_per_node, max_nodes_x, max_nodes_y, nodes, connections, entrance_node_id, finish_node_id

data = {
    "World": {
        "OccupancyGridMap": {
            "filename": "../maps/maze_small.png",
            "resolution": 0.250000,
            "pixels_per_node": 2,
            "max_nodes_x": 10,
            "max_nodes_y": 10
        }
    },
  "nodes": [
    [2, 1],
    [2, 2],
    [2, 3],
    [2, 4],
    [2, 5],
    [2, 6],
    [2, 7],
    [2, 8],
    [2, 9],
    [3, 3],
    [3, 9],
    [4, 2],
    [4, 3],
    [4, 5],
    [4, 6],
    [4, 7],
    [5, 2],
    [5, 5],
    [5, 7],
    [5, 8],
    [5, 9],
    [6, 2],
    [6, 3],
    [6, 4],
    [6, 5],
    [6, 9],
    [7, 2],
    [7, 5],
    [7, 7],
    [7, 8],
    [7, 9],
    [8, 2],
    [8, 5],
    [8, 7],
    [9, 2],
    [9, 3],
    [9, 4],
    [9, 5],
    [9, 7],
    [9, 8],
    [9, 9],
    [10, 8]
  ],
 "connections": [
    [1],
    [0, 2],
    [9, 1, 3],
    [2, 4],
    [3, 5],
    [4, 6],
    [5, 7],
    [6, 8],
    [10, 7],
    [2, 12],
    [8],
    [16, 12],
    [9, 11],
    [17, 14],
    [13, 15],
    [18, 14],
    [11, 21],
    [13, 24],
    [15, 19],
    [18, 20],
    [25, 19],
    [16, 26, 22],
    [21, 23],
    [22, 24],
    [17, 27, 23],
    [20, 30],
    [21, 31],
    [24, 32],
    [33, 29],
    [28, 30],
    [25, 29],
    [26, 34],
    [27, 37],
    [28, 38],
    [31, 35],
    [34, 36],
    [35, 37],
    [32, 36],
    [33, 39],
    [41, 38, 40],
    [39],
    [39]
  ],
    "entrance": 0,
    "finish": 41
}


planner = AstarPlanner(data)


# Get the nodelist
nodelist = planner.nodelist
finish_node_id = planner.finish_node_id
entrance_node_id = planner.entrance_node_id
connections = planner.connections

# Plan a path
# 0: Initialization
n_nodes = len(nodelist)
print(n_nodes)