#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Pose2DStamped
from duckietown_msgs.msg import Twist2DStamped 
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import signal
import sys
import cv2
import json
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
    def __init__(self, config_file):
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

        # Read the JSON config file
        with open(config_file) as file:
            configfile = json.load(file)

        if 'nodes' in configfile:
            self.n_nodes = len(configfile['nodes'])

        self.nodelist = [Node() for _ in range(self.n_nodes)]
        self.connections = [[] for _ in range(self.n_nodes)]

        # Extract relevant information from the config file
        if 'World' in configfile and 'OccupancyGridMap' in configfile['World']:
            occupancy_grid_map = configfile['World']['OccupancyGridMap']
            self.max_nodes_x = occupancy_grid_map.get('max_nodes_x', 0)
            self.max_nodes_y = occupancy_grid_map.get('max_nodes_y', 0)
            self.resolution = occupancy_grid_map.get('resolution', 0.0)
            self.pixels_per_node = occupancy_grid_map.get('pixels_per_node', 0.0)
            self.node_resolution = self.resolution * self.pixels_per_node
            print("Resolution:", self.resolution)

        if 'nodes' in configfile and 'entrance' in configfile and 'finish' in configfile and 'connections' in configfile:
            nodes = configfile['nodes']
            self.entrance_node_id = configfile['entrance']
            self.finish_node_id = configfile['finish']

            for i_node in range(self.n_nodes):
                new_node = Node()
                new_node.x = self.node_resolution * (0.5 * self.max_nodes_x - (nodes[i_node][0] - 0.5))
                new_node.y = self.node_resolution * (0.5 * self.max_nodes_y - (nodes[i_node][1] - 0.5))
                new_node.g = float('inf')
                new_node.h = 0.0
                new_node.f = new_node.g + new_node.h
                self.nodelist[i_node] = new_node

                self.connections[i_node] = configfile['connections'][i_node]

            print("Entrance Node ID:", self.entrance_node_id)
            print("Exit Node ID:", self.finish_node_id)
            print("Connections:", self.connections[1][1])


# Example usage

planner = AstarPlanner("/code/catkin_ws/src/We-will-do-it/packages/my_package/config/params_maze_small.json")



# Get the nodelist
nodelist = planner.nodelist
finish_node_id = planner.finish_node_id
entrance_node_id = planner.entrance_node_id
connections = planner.connections

# Plan a path
# 0: Initialization
n_nodes = len(nodelist)
node_goal = nodelist[finish_node_id]
for nodeID in range(n_nodes):
    nodelist[nodeID].g = float('inf')  # cost-to-come from the start to this node
    nodelist[nodeID].h = calculate_distance(nodelist[nodeID], node_goal)  # heuristic cost-to-go from this node to the goal
    nodelist[nodeID].f = nodelist[nodeID].g + nodelist[nodeID].h  # cost-to-come + cost-to-go
    nodelist[nodeID].parent_node_ID = NODE_ID_NOT_SET  # ID of the node from which node you arrived at this node with the lowest cost-to-come

open_nodes = [entrance_node_id]
closed_nodes = []
nodelist[entrance_node_id].g = 0.0
nodelist[entrance_node_id].f = nodelist[entrance_node_id].g + nodelist[entrance_node_id].h
goal_reached = False

# 1: Plan path using the A* algorithm
while not goal_reached and open_nodes:
    # Find the node number from the ones that are open with minimum f
    nodeID_minimum_f = NODE_ID_NOT_SET
    min_f = float('inf')
    for i in open_nodes:
        if nodelist[i].f < min_f:
            min_f = nodelist[i].f
            nodeID_minimum_f = i

    # The goal is reached (and via the optimal path) when the goal is the open node with minimum f
    if nodeID_minimum_f == finish_node_id:
        goal_reached = True
    elif nodeID_minimum_f == NODE_ID_NOT_SET:
        break
    else:
        current_nodeID = nodeID_minimum_f

        # Explore the nodes connected to the new current node if they were not closed yet
        for neighbour in connections[current_nodeID]:
            if neighbour not in closed_nodes and neighbour not in open_nodes:
                open_nodes.append(neighbour)
                nodelist[neighbour].g = calculate_distance(nodelist[neighbour], nodelist[current_nodeID]) + nodelist[current_nodeID].g
                nodelist[neighbour].f = nodelist[neighbour].g + nodelist[neighbour].h
                nodelist[neighbour].parent_node_ID = current_nodeID
            elif neighbour not in closed_nodes and neighbour in open_nodes:
                new_g = calculate_distance(nodelist[neighbour], nodelist[current_nodeID]) + nodelist[current_nodeID].g
                new_f = new_g + nodelist[neighbour].h
                new_parent = current_nodeID
                if nodelist[neighbour].f > new_f:
                    # update g, f, parent
                    nodelist[neighbour].f = new_f
                    nodelist[neighbour].g = new_g
                    nodelist[neighbour].parent_node_ID = new_parent

        # remove the current node from the open list and add it to the closed list
        open_nodes.remove(current_nodeID)
        closed_nodes.append(current_nodeID)

# 2: Trace back the optimal path (if the goal could be reached)
if goal_reached:
    path_node_IDs = []
    nodeID = finish_node_id
    while nodeID != entrance_node_id:
        path_node_IDs.insert(0, nodeID)
        nodeID = nodelist[nodeID].parent_node_ID
    
    path_node_IDs.insert(0, entrance_node_id)

    # Print the path node IDs
    print("Optimal Path Node IDs:")
    for nodeID in path_node_IDs:
        print(nodeID)

for i in range(len(path_node_IDs)):
    x_next, y_next = nodelist[path_node_IDs[i]].x, nodelist[path_node_IDs[i]].y
    print(x_next,y_next)

class MyAstarNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyAstarNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        # construct publisher and subscriber
        self.i = 1
        self.pub = rospy.Publisher('/db8/joy_mapper_node/car_cmd', Twist2DStamped, queue_size=10)
        if self.i == 1:
            self.sub = rospy.Subscriber('/db8/velocity_to_pose_node/pose', Pose2DStamped, self.get_values)
        

    # SUbcriber :- Odometry Data 
    def get_values(self, data):
        odom_x = data.x
        odom_y = data.y
        odom_theta = data.theta
        rospy.loginfo("Odometry: x=%.2f, y=%.2f, theta=%.2f", odom_x, odom_y, odom_theta)
        distance_threshold = 0.03  # [m]
        err_o_threshold = 3  # [rad]
        vel_rotate = 2.0  # [rad/s]
        vel_translate = 0.3  # [m/s]
        gain_rotate_while_translate = 10

        # Move to the locations specified in the path one-by-one
        x_next, y_next = nodelist[path_node_IDs[self.i]].x, nodelist[path_node_IDs[self.i]].y
        err_x = odom_x - x_next
        err_y = odom_y - y_next
        err_o = self.wrapToPi(odom_theta - math.atan2(y_next - odom_y, x_next - odom_x))
        if err_x ** 2 + err_y ** 2 >= distance_threshold ** 2:
            # Read odometry data and update pose error calculations
            if err_o >= err_o_threshold or err_o <= -err_o_threshold:
                print("I am rotating")
                # Orient towards next point    
                omega = -vel_rotate * err_o / abs(err_o)
                self.run(0,omega)
                rospy.loginfo("Odometry new in simulate_path_following: x=%.2f, y=%.2f, theta=%.2f", odom_x, odom_y, odom_theta)
                rospy.Subscriber('/db8/velocity_to_pose_node/pose', Pose2DStamped, self.get_values)
            else:
                # Move forward and keep orientation correct
                print("I am moving straight")
                self.run(vel_translate,-gain_rotate_while_translate*err_o)                  #-gain_rotate_while_translate*err_o)
                rospy.loginfo("Odometry new in simulate_path_following: x=%.2f, y=%.2f, theta=%.2f", odom_x, odom_y, odom_theta)
                rospy.Subscriber('/db8/velocity_to_pose_node/pose', Pose2DStamped, self.get_values)
        else:
            self.i +=1
            rospy.Subscriber('/db8/velocity_to_pose_node/pose', Pose2DStamped, self.get_values)
   

    def wrapToPi(self,angle):
        if angle > math.pi:
            k = math.ceil(angle / (2 * math.pi))
            angle -= 2 * k * math.pi
            angle = self.wrapToPi(angle)  # Rerun to ensure correctness
        elif angle < -math.pi:
            k = math.floor(angle / (2 * math.pi))
            angle += 2 * math.pi
            angle = self.wrapToPi(angle)  # Rerun to ensure correctness
        return angle

    def run(self,velocity, omega): 
        ## publish message every 1 second
        rate = rospy.Rate(1)
        twist = Twist2DStamped()
        twist.header = Header()
        twist.header.stamp = rospy.Time.now()
        twist.header.frame_id = " "
        twist.v = velocity # Set the linear velocity
        twist.omega = omega # Set the angular velocity
        rospy.loginfo("Publishing message: '%s'" % twist)
        self.pub.publish(twist)


if __name__ == '__main__':
    node = MyAstarNode(node_name='astartbasic')
    rospy.spin()