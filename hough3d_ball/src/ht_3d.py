#!/usr/bin/env python3

'''
Implements the 3D Hough in several ways (vanilla, stochastic, refinement)
'''

import numpy as np

def hough3d(pointcloud : np.array, ball_d : float, refine : False, stochastic : False) -> np.array:
    '''
    Returns the center of a ball of diameter ball_d that is detected in pointcloud
    '''
    raise NotImplementedError