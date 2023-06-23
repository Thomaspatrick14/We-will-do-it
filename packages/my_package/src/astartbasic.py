#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Pose2DStamped  # odometry message type (for x,y,theta)
from duckietown_msgs.msg import Twist2DStamped  # motor command message type ( for v, omega) 
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import signal
import sys
import cv2

class MyAstarNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyAstarNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        # construct publisher and subscriber
        #self.pub = rospy.Publisher('/db8/velocity', Twist2DStamped, queue_size=10)  # Replace with your motor command topic
        self.pub = rospy.Publisher('/db8/joy_mapper_node/car_cmd', Twist2DStamped, queue_size=10)
        self.sub = rospy.Subscriber('/db8/velocity_to_pose_node/pose', Pose2DStamped, self.callback)  # Replace with your odometry topic
        self.stop_flag = False
        
    def callback(self, data):
       x = data.x
       y = data.y
       theta = data.theta
       rospy.loginfo("Odometry: x=%.2f, y=%.2f, theta=%.2f", x, y, theta)

        
    def run(self): 
    #     # publish message every 1 second
        rate = rospy.Rate(1) # 1Hz
       

        while not rospy.is_shutdown() and not self.stop_flag:
            twist = Twist2DStamped()
            twist.header = Header()
            twist.header.stamp = rospy.Time.now()
            twist.header.frame_id = " "
            twist.v = 0.3  # Set the linear velocity
            twist.omega = 3 # Set the angular velocity
            rospy.loginfo("Publishing message: '%s'" % twist)
            self.pub.publish(twist)
            rate.sleep()

            

    # Check for Ctrl+C signal
            if cv2.waitKey(1) == 3:  # ASCII code for Ctrl+C is 3
                self.stop_bot()
    
    def stop_bot(self):
        self.stop_flag = True

        # while not rospy.is_shutdown():
        #     twist = Twist()
        #     twist.linear.x = 0.1  # Set the linear velocity
        #     twist.angular.z = 0.0  # Set the angular velocity
        #     rospy.loginfo("Publishing twist command: %s" % twist)
        #     self.pub.publish(twist)
        #     rate.sleep()
        

    
if __name__ == '__main__':
    
    node = MyAstarNode(node_name='astartbasic')
    #signal.signal(signal.SIGINT, node.keyboard_interrupt_handler)
    
    try: 
    # create the node
    # run the node
       node.run()
  
    # keep spinning
    #rospy.spin()
    except rospy.ROSInterruptException:
        pass
    