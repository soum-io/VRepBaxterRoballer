# -*- coding: utf-8 -*-
"""
Created on Sun Mar 11 16:13:17 2018

@author: Jacob
"""
# this function opens communications with the vrep simulator and
# defines the joints for the Baxter robot in vrep.
# returns: clientID, clientID, bodyJoints, rightArm, leftArm
def startup():
    import vrep
    import numpy as np

    # Close all open connections (just in case)
    vrep.simxFinish(-1)

    # Connect to V-REP (raise exception on failure)
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID == -1:
        raise Exception('Failed connecting to remote API server')


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
    
    return clientID, bodyJoints, rightArm, leftArm
