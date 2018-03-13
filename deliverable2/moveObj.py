# -*- coding: utf-8 -*-
"""
Created on Mon Mar 12 19:01:31 2018

@author: Jacob
"""

# this function places a dummy object in the Vrep simulation given a pose of the
# following format: T = [R p] 
#                       [0 1]
# the pose should be a numpy array type 
import vrep
import math
import numpy as np


def moveObj(T, clientID, objHandle, bodyJoints):    
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
    x = 0
    y = 0
    z = 0
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    E = np.array([x, y, z])
    
    vrep.simxSetObjectPosition(clientID, objHandle, -1, p, vrep.simx_opmode_streaming)
    vrep.simxSetObjectOrientation(clientID, objHandle, -1, E, vrep.simx_opmode_oneshot)
    
    
    
    