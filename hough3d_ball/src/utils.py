#!/usr/bin/env python3
'''
Utility functions for the 3D hough transform library 
'''

import numpy as np
import rospy
import ctypes
import struct
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


def convert_pcloud_to_np(pointcloud_msg : PointCloud2):
    '''
    Converts ROS pointcloud2 msgs to two numpy arrays (one for xyz and one for rgb)
    '''
    xyz = np.tile(np.zeros((1,3)),(pointcloud_msg.width,1))
    rgb = np.tile(np.zeros((1,3)),(pointcloud_msg.width,1))
    #self.lock.acquire()
    gen = pc2.read_points(pointcloud_msg, skip_nans=True)
    int_data = list(gen)

    for idx,x in enumerate(int_data):
        test = x[3] 
        # cast float32 to int so that bitwise operations are possible
        s = struct.pack('>f' ,test)
        i = struct.unpack('>l',s)[0]
        # you can get back the float value by the inverse operations
        pack = ctypes.c_uint32(i).value
        r = (pack & 0x00FF0000)>> 16
        g = (pack & 0x0000FF00)>> 8
        b = (pack & 0x000000FF)
        # prints r,g,b values in the 0-255 range
        # x,y,z can be retrieved from the x[0],x[1],x[2]
        xyz[idx, :] = np.array([[x[0],x[1],x[2]]])
        rgb[idx, :] = np.array([[r,g,b]])
    
    return xyz, rgb
    
