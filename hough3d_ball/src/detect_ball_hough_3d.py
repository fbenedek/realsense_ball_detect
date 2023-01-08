#!/usr/bin/env python3

'''
Runs a 3D Hough transform to detect balls of a known diameter in a pointcloud message.
'''
# standard
import os
import sys
import time 
import ctypes
import struct

# ROS
import queue
import roslib 
import rospy
from sensor_msgs.msg import PointCloud2 
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Header

# other third-party
# import matplotlib.pyplot as plt
import numpy as np

# custom
from utils import convert_pcloud_to_np
# from ht_3d import hough3d

DEBUG_NODE = True
BALL_DIAMETER = 0.1

class BallDetector:
    '''
    Takes in the center of the largest cluster on an image and tracks it using RANSAC + KF. Publishes the ball state @ 100 Hz.
    '''
    def __init__(self):
        self.pointcloud_sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.process_measurement)
        self.detected_ball_position_pub = rospy.Publisher("/hough3d/ball_detection", PointStamped, queue_size=1)
        self.debug_node = DEBUG_NODE
        self.ball_diameter = BALL_DIAMETER
        self.use_probabilistic_method = False
        self.refinement = False
    

    def process_measurement(self, pointcloud_msg):
        '''
        Processes an incoming cluster center 3D point
        '''
        print(f"Incoming pointcloud with dimensions {pointcloud_msg.width}x{pointcloud_msg.height}")
        xyz, rgb = convert_pcloud_to_np(pointcloud_msg)        
        print(f"Converted the point coordinates to a numpy array with dimensions {xyz.shape}")
        # TODO implement 3D hough trf

    
    def process_measurement(self, )
    
if __name__ == '__main__':
    rospy.loginfo("Node init...")
    ball_detector = BallDetector()
    rospy.init_node('ball_tracker_node', anonymous=True)
    #rospy.Timer(rospy.Duration(1.0/ransac_tracker.ball_pub_rate), ransac_tracker.publish_ball_state)
    rospy.loginfo("Node init ran")
    rospy.spin()