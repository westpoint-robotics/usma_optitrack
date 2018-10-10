#!/usr/bin/env python

# Import required Python code.
import math
import sys
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Vector3, Twist #TwistStamped
import time
import tf

class vrpn_logger():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # ros config
        self.pose_topic = rospy.get_param("~pose_msg","/vrpn_client_node/Ardrone/pose")
        self.topic_sub = rospy.Subscriber(self.pose_topic,PoseStamped,self.mocap_pose_cb)
        # matlab config
        self.matlab_prefix = rospy.get_param("~matlab_prefix","vrpnlog")
        self.msg_seq = 0
        # write file config
        self.data_logger_filepath = rospy.get_param("~filepath","/home/benjamin/ros/data/")
        self.data_logger_filename = self.data_logger_filepath + self.matlab_prefix + ".m"
        self.data_logger = open(self.data_logger_filename, 'w')
        self.data_logger.write(("%%filename: {} \n\n").format(self.data_logger_filename))

    def mocap_pose_cb(self,msg):
        msg_time = msg.header.stamp.to_sec()
        self.msg_seq += 1
        orientation = msg.pose.orientation
        position = msg.pose.position
        self.data_logger.write(("{}.pose.time({},1) = [{}];\n").format(self.matlab_prefix, self.msg_seq, msg_time))
        self.data_logger.write(("{}.pose.linear({},:) = [{:06.8f} {:06.8f} {:06.8f}];\n").format(self.matlab_prefix, self.msg_seq, position.x, position.y, position.z))
        self.data_logger.write(("{}.pose.angular({},:) = [{:06.8f} {:06.8f} {:06.8f} {:06.8f}];\n\n").format(self.matlab_prefix, self.msg_seq, orientation.x, orientation.y, orientation.z, orientation.w))

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('vrpn_logger')
    try:
        vrpnlog = vrpn_logger()
    except rospy.ROSInterruptException: pass

    dummy = None
    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():

        rate.sleep()

# end main

