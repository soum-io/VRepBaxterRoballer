'''
This file contains all the variables that will be used in the final video 
diliverable. These variables are used in functions.py and Final Motions.py
'''

import numpy as np
import scipy as sp
from numpy import linalg as nl
from scipy import linalg as sl
import math
from math import factorial
import vrep
import time
import random
import copy

    
#end previous simulations if there are any
vrep.simxFinish(-1)

#create new client for simulation
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')

# define the body joints of baxter
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


#define object variables in scene that baxter can interact with
block0err, block0 = vrep.simxGetObjectHandle(clientID, "cube0", vrep.simx_opmode_blocking)
block2err, block2 = vrep.simxGetObjectHandle(clientID, "cube2", vrep.simx_opmode_blocking)
block1err, block1 = vrep.simxGetObjectHandle(clientID, "cube1", vrep.simx_opmode_blocking)
fireerr, fire = vrep.simxGetObjectHandle(clientID, "fire", vrep.simx_opmode_blocking)

#used to be smoke, but now these objects are tiles to indicate where the postions are 
#in tower of Hanoi
smoke0err, smoke0 = vrep.simxGetObjectHandle(clientID, "smoke#0", vrep.simx_opmode_blocking)
smoke1err, smoke1 = vrep.simxGetObjectHandle(clientID, "smoke#1", vrep.simx_opmode_blocking)
smoke2err, smoke2 = vrep.simxGetObjectHandle(clientID, "smoke#2", vrep.simx_opmode_blocking)


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


#initial Poses for both of Baxter's arms

#left arm
ML = np.array([[1,0,0,.0815],
                  [0,1,0,1.3531],
                  [0,0,1,1.2454],
                  [0,0,0,1]])
#right arm
MR = np.array([[1,0,0,.0819],
                  [0,1,0,-1.3466],
                  [0,0,1,1.2454],
                  [0,0,0,1]])

#this is a command to communicate with the lua scripts (v-rep programming language)
#of both the sctuon cups baxter uses. Tead the lua scripts attached to baxters suction
#cups to get a better understanding 
leftcup = "BaxterVacuumCup_active"
rightcup = "BaxterVacuumCup#0_active"

#there will all be used as dummy variables that will be attached to baxter's joints
#and arm tips for collision purposed and vizualization purposes
objHandleLeftTheoretical = int(vrep.simxGetObjectHandle(clientID, 'ReferenceFrame', vrep.simx_opmode_blocking)[1])
objHandleRightTheoretical = int(vrep.simxGetObjectHandle(clientID, 'ReferenceFrame0', vrep.simx_opmode_blocking)[1])
dummyRot = int(vrep.simxGetObjectHandle(clientID, 'DummyRot', vrep.simx_opmode_blocking)[1])
dummy1L = int(vrep.simxGetObjectHandle(clientID, 'Dummy1L', vrep.simx_opmode_blocking)[1])
dummy2L = int(vrep.simxGetObjectHandle(clientID, 'Dummy2L', vrep.simx_opmode_blocking)[1])
dummy3L = int(vrep.simxGetObjectHandle(clientID, 'Dummy3L', vrep.simx_opmode_blocking)[1])
dummy4L = int(vrep.simxGetObjectHandle(clientID, 'Dummy4L', vrep.simx_opmode_blocking)[1])
dummy5L = int(vrep.simxGetObjectHandle(clientID, 'Dummy5L', vrep.simx_opmode_blocking)[1])
dummy6L = int(vrep.simxGetObjectHandle(clientID, 'Dummy6L', vrep.simx_opmode_blocking)[1])
dummy7L = int(vrep.simxGetObjectHandle(clientID, 'Dummy7L', vrep.simx_opmode_blocking)[1])
dummyEndL = int(vrep.simxGetObjectHandle(clientID, 'DummyEndL', vrep.simx_opmode_blocking)[1])
dummyOutside = int(vrep.simxGetObjectHandle(clientID, 'DummyOutside', vrep.simx_opmode_blocking)[1])
dummy1R = int(vrep.simxGetObjectHandle(clientID, 'Dummy1R', vrep.simx_opmode_blocking)[1])
dummy2R = int(vrep.simxGetObjectHandle(clientID, 'Dummy2R', vrep.simx_opmode_blocking)[1])
dummy3R = int(vrep.simxGetObjectHandle(clientID, 'Dummy3R', vrep.simx_opmode_blocking)[1])
dummy4R = int(vrep.simxGetObjectHandle(clientID, 'Dummy4R', vrep.simx_opmode_blocking)[1])
dummy5R = int(vrep.simxGetObjectHandle(clientID, 'Dummy5R', vrep.simx_opmode_blocking)[1])
dummy6R = int(vrep.simxGetObjectHandle(clientID, 'Dummy6R', vrep.simx_opmode_blocking)[1])
dummy7R = int(vrep.simxGetObjectHandle(clientID, 'Dummy7R', vrep.simx_opmode_blocking)[1])
dummyEndR = int(vrep.simxGetObjectHandle(clientID, 'DummyEndR', vrep.simx_opmode_blocking)[1])
dummyPos = int(vrep.simxGetObjectHandle(clientID, 'DummyPos', vrep.simx_opmode_blocking)[1])

#create an array of the dummy joints for ease of use
leftArmDummies = np.array([dummyRot, dummy1L, dummy2L, dummy3L, dummy4L, dummy5L, dummy6L, dummy7L, dummyEndL])
rightArmDummies = np.array([dummyRot, dummy1R, dummy2R, dummy3R, dummy4R, dummy5R, dummy6R, dummy7R, dummyEndR])

#arm joint angle limits (used in collision functions to ensure inver kinamatics equations 
#does not come up with a set of theta's that baxter's joints cant achieve)
rightLimits = [(math.radians(-168), math.radians(-168+336)), (math.radians(-56), math.radians(-56+152.5)), (math.radians(-115), math.radians(-115+175)), (math.radians(-173), math.radians(-173+346)), (math.radians(0), math.radians(0+148)), (math.radians(-173.3), math.radians(-173.3+346.5)), (math.radians(-88), math.radians(-88+206)),(math.radians(-173.3), math.radians(-173.3+346.5))]
LeftLimits = [(math.radians(-168), math.radians(-168+336)), (math.radians(-97.5), math.radians(-97.5+152.5)), (math.radians(-115), math.radians(-115+175)), (math.radians(-173), math.radians(-173+346)), (math.radians(0), math.radians(0+148)), (math.radians(-173.3), math.radians(-173.3+346.5)), (math.radians(-88), math.radians(-88+206)),(math.radians(-173.3), math.radians(-173.3+346.5))]
(math.radians(-97.5), math.radians(-97.5+152.5))

#outside obstacle properties
#postion
p_obstacle = np.array([[ 0, 0, 0], [ 0, 0, 0], [ 1.6, 1.25, .4]])
#radius
r_obstacle = np.array([[.2, .15, .2]])

#start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

#original centers of cubes in tower of Hanoi - not used currently
cubeCenters = np.array([[.7,.7,.7],
                        [0,-.3,.3],
                        [.715,.715,.715]])

