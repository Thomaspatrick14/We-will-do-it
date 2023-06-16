#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from custom_msgs.msg import OdometryMsg  # Replace with your odometry message type
from custom_msgs.msg import MotorCommandMsg  # Replace with your motor command message type

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


class MyAstarNode(DTROS):

    def __init__(self, node_name, nodelist, connections):
        # initialize the DTROS parent class
        super(MyAstarNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # construct publisher and subscriber
        self.pub = rospy.Publisher('motor_cmd', MotorCommandMsg, queue_size=10)  # Replace with your motor command topic
        self.sub = rospy.Subscriber('odom_data', OdometryMsg, self.callback)  # Replace with your odometry topic

        # store the nodelist and connections as instance variables
        self.nodelist = nodelist
        self.connections = connections

        # Load config and initialize variables
        self.load_config()
        self.initialize_astar()

    def load_config(self):
        # Read the JSON config file
        config_filename = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../config/params_maze_small.json')
        with open(config_filename) as file:
            configfile = json.load(file)

        # Extract relevant information from the config file and set class variables
        if 'World' in configfile and 'OccupancyGridMap' in configfile['World']:
            occupancy_grid_map = configfile['World']['OccupancyGridMap']
            self.max_nodes_x = occupancy_grid_map.get('max_nodes_x', 0)
            self.max_nodes_y = occupancy_grid_map.get('max_nodes_y', 0)
            self.resolution = occupancy_grid_map.get('resolution', 0.0)
            self.pixels_per_node = occupancy_grid_map.get('pixels_per_node', 0.0)
            self.node_resolution = self.resolution * self.pixels_per_node

        if 'nodes' in configfile and 'entrance' in configfile and 'finish' in configfile and 'connections' in configfile:
            nodes = configfile['nodes']
            self.entrance_node_id = configfile['entrance']
            self.finish_node_id = configfile['finish']

            self.nodelist = [Node() for _ in range(len(nodes))]
            self.connections = [[] for _ in range(len(nodes))]

            for i_node in range(len(nodes)):
                new_node = Node()
                new_node.x = self.node_resolution * (0.5 * self.max_nodes_x - (nodes[i_node][0] - 0.5))
                new_node.y = self.node_resolution * (0.5 * self.max_nodes_y - (nodes[i_node][1] - 0.5))
                new_node.g = float('inf')
                new_node.h = 0.0
                new_node.f = new_node.g + new_node.h
                self.nodelist[i_node] = new_node
                self.connections[i_node] = configfile['connections'][i_node]

    def callback(self, data):
        # Process the received odometry data
        # Extract relevant information from the odometry message
        # Perform your computations or control logic based on the odometry data
        # Access the nodelist and connections
        
        
        # Example: extract linear velocity
        linear_velocity = data.linear_velocity

        # Example: calculate desired motor command based on odometry data
        motor_command = linear_velocity * 2.0  # Just an example calculation, replace with your own control logic

        # Create and publish the motor command
        command_msg = MotorCommandMsg()
        command_msg.command = motor_command

        rospy.loginfo("Publishing motor command: %.2f" % command_msg.command)
        self.pub.publish(command_msg)

    def run(self):
        # Keep spinning
        rospy.spin()


if __name__ == '__main__':
    # Create the node
    node = MyAstarNode(node_name='my_astar_node')
    # Run the node
    node.run()
