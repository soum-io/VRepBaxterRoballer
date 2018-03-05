# -*- coding: utf-8 -*-
"""
Created on Sun Mar  4 15:59:35 2018

@author: Jacob
"""
import vrep
import time
import numpy as np

# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')
'''
# Get "handle" to the first joint of robot
result, joint_one_handle = vrep.simxGetObjectHandle(clientID, 'UR3_joint1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get object handle for first joint')
'''

## define Baxter's joints
# define the body joints
bodyJoints = np.zeros(3)
bodyError = np.zeros(3)
vertError, vertJoint = vrep.simxGetObjectHandle(clientID, 'Baxter_verticalJoint', vrep.simx_opmode_blocking)
rotError, rotJoint = vrep.simxGetObjectHandle(clientID, 'Baxter_rotationJoint', vrep.simx_opmode_blocking)
monitorError, monitorJoint = vrep.simxGetObjectHandle(clientID, 'Baxter_monitorJoint', vrep.simx_opmode_blocking)

bodyJoints[0]= vertJoint
bodyError[0] = vertError
bodyJoints[1]= rotJoint
bodyError[1] = rotError
bodyJoints[2]= monitorJoint
bodyError[2] = monitorError
bodyJoints = bodyJoints.astype(int)
bodyError = bodyError.astype(int)

# define the arm joints
# the number of joints per arm
num_joints = 7
i = 0
rightArm = np.zeros(num_joints)
rightError = np.zeros(num_joints)
leftArm = np.zeros(num_joints)
leftError = np.zeros(num_joints)

for i in range(7):
    # define the joint names we ask from v-rep
    rightJointName = 'Baxter_rightArm_joint' + str(i+1)
    leftJointName = 'Baxter_leftArm_joint' + str(i+1)
    
    # get the joint names from v-rep
    rightError[i], rightArm[i] = vrep.simxGetObjectHandle(clientID, rightJointName, vrep.simx_opmode_blocking)
    leftError[i], leftArm[i] = vrep.simxGetObjectHandle(clientID, leftJointName, vrep.simx_opmode_blocking)
rightArm = rightArm.astype(int)
rightError = rightError.astype(int)
leftArm = leftArm.astype(int)
leftError = leftError.astype(int)

# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# Wait two seconds
time.sleep(2)

'''
# starting position
for i in range(7):
    vrep.simxSetJointTargetPosition(clientID, rightArm[i], np.deg2rad(30), vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, leftArm[i], np.deg2rad(30), vrep.simx_opmode_oneshot)
for i in range(3):
    vrep.simxSetJointTargetPosition(clientID, bodyJoints[i], np.deg2rad(30), vrep.simx_opmode_oneshot)
'''
# move the robot joints
for i in range(3):
    vrep.simxSetJointTargetPosition(clientID, bodyJoints[i], (np.pi / 2), vrep.simx_opmode_oneshot)

    time.sleep(2)
    
    vrep.simxSetJointTargetPosition(clientID, bodyJoints[i], 0, vrep.simx_opmode_oneshot)
    
    time.sleep(2)

for i in range(7):
    vrep.simxSetJointTargetPosition(clientID, rightArm[i], (np.pi / 2), vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, leftArm[i], (np.pi / 2), vrep.simx_opmode_oneshot)
    
    time.sleep(.5)
    
    vrep.simxSetJointTargetPosition(clientID, rightArm[i], 0, vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, leftArm[i], 0, vrep.simx_opmode_oneshot)
    
    time.sleep(.5)

for i in range(7):
    vrep.simxSetJointTargetPosition(clientID, rightArm[i], 0, vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, leftArm[i], 0, vrep.simx_opmode_oneshot)
    
    time.sleep(.5)
    
    vrep.simxSetJointTargetPosition(clientID, rightArm[i], (np.pi / 2), vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, leftArm[i], (np.pi / 2), vrep.simx_opmode_oneshot)
    
    time.sleep(.5)

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)

















