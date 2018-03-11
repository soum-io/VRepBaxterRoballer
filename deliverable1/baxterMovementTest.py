# -*- coding: utf-8 -*-
"""
Created on Sun Mar  4 15:59:35 2018

@author: Jacob
"""
import vrep
import time
import numpy as np
from startup import startup

# open comms with the simulator and define the robot joints
clientID, bodyJoints, rightArm, leftArm = startup()

# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# Wait two seconds
time.sleep(2)

# starting position
print('Moving to Initial Position')
for i in range(7):
    vrep.simxSetJointTargetPosition(clientID, rightArm[i], 0, vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, leftArm[i], 0, vrep.simx_opmode_oneshot)

# move the robot joints
for i in range(3):    
    vrep.simxSetJointTargetPosition(clientID, bodyJoints[i], np.deg2rad(15), vrep.simx_opmode_oneshot)
    time.sleep(2)
    vrep.simxSetJointTargetPosition(clientID, bodyJoints[i], 0, vrep.simx_opmode_oneshot)
    time.sleep(2)

for i in range(7):
    vrep.simxSetJointTargetPosition(clientID, rightArm[i], (np.pi / 2), vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, leftArm[i], (np.pi / 2), vrep.simx_opmode_oneshot)
    
    time.sleep(2)
    
    vrep.simxSetJointTargetPosition(clientID, rightArm[i], 0, vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, leftArm[i], 0, vrep.simx_opmode_oneshot)
    
    time.sleep(2)

vrep.simxSetJointTargetPosition(clientID, bodyJoints[1], 190, vrep.simx_opmode_oneshot)
time.sleep(5)

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)
