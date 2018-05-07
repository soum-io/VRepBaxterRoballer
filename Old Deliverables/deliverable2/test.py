# -*- coding: utf-8 -*-
"""
Created on Mon Mar 12 19:49:04 2018

@author: Jacob
"""
import vrep
import numpy as np
from startup import startup
from moveObj import moveObj
import time

# open comms with the simulator and define the robot joints
clientID, bodyJoints, rightArm, leftArm = startup()

# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

objHandle = int(vrep.simxGetObjectHandle(clientID, 'ReferenceFrame', vrep.simx_opmode_blocking)[1])
T = np.array([[1, 0, 0, .5], [0, 1, 0, .5], [0, 0, 1, .5], [0, 0, 0, 1]])
moveObj(T, clientID, objHandle)

time.sleep(10)
# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)









