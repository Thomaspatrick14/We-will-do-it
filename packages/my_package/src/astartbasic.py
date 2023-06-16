#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from custom_msgs.msg import OdometryMsg  # Replace with your odometry message type
from custom_msgs.msg import MotorCommandMsg  # Replace with your motor command message type

class MyAstarNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyAstarNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        # construct publisher and subscriber
        self.pub = rospy.Publisher('wheels', MotorCommandMsg, queue_size=10)  # Replace with your motor command topic
        self.sub = rospy.Subscriber('odom_data', OdometryMsg, self.callback)  # Replace with your odometry topic
        
    def callback(self, data):
        # process the received odometry data
        
        # extract relevant information from the odometry message
        # perform your computations or control logic based on the odometry data
        
        # example: extract linear velocity
        linear_velocity = data.linear_velocity
        
        # example: calculate desired motor command based on odometry data
        motor_command = linear_velocity * 2.0  # just an example calculation, replace with your own control logic
        
        # create and publish the motor command
        command_msg = MotorCommandMsg()
        command_msg.command = motor_command
        
        rospy.loginfo("Publishing motor command: %.2f" % command_msg.command)
        self.pub.publish(command_msg)
        
    def run(self):
        # keep spinning
        rospy.spin()

if __name__ == '__main__':
    # create the node
    node = MyAstarNode(node_name='my_astar_node')
    # run the node
    node.run()


