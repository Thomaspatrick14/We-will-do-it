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

# Example usage:
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

resolution, pixels_per_node, max_nodes_x, max_nodes_y, nodes, connections, entrance_node_id, finish_node_id = extract_data_from_dict(data)


# print("Resolution:", resolution)
# print("Pixels per node:", pixels_per_node)
# print("Max nodes x:", max_nodes_x)
# print("Max nodes y:", max_nodes_y)
# print("Nodes:", nodes)
# print("Connections:", connections)
# print("Entrance node ID:", entrance_node_id)
# print("Finish node ID:", finish_node_id)
