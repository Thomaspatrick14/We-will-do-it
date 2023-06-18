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
        # odometry data
        odom = data.x
        rospy.loginfo("Subscriber the odometry: %.2f" %odom)
        
    def run(self): 
        # publish message every 1 second
        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():
            linear_velocity = 0.1
            rospy.loginfo("Publishing message: '%s'" % linear_velocity)
            self.pub.publish(linear_velocity)
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = MyAstarNode(node_name='my_astar_node')
    # run the node
    node.run()
    # keep spinning
    rospy.spin()


