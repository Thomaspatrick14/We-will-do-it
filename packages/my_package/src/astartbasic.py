#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Pose2DStamped  # odometry message type (for x,y,theta)
from duckietown_msgs.msg import Twist2DStamped  # motor command message type ( for v, omega) 

class MyAstarNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyAstarNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        # construct publisher and subscriber
        self.pub = rospy.Publisher('wheels', Twist2DStamped, queue_size=10)  # Replace with your motor command topic
        self.sub = rospy.Subscriber('odom_data', Pose2DStamped, self.callback)  # Replace with your odometry topic
        
    def callback(self, data):
        # process the received odometry data
        
        # extract relevant information from the odometry message
        # perform your computations or control logic based on the odometry data
        
        # example: extract linear velocity
        linear_velocity = data.v
        
        # example: calculate desired motor command based on odometry data
        motor_command = linear_velocity * 0.1 # just an example calculation, replace with your own control logic
        
        # create and publish the motor command
        command_msg = Twist2DStamped()
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


