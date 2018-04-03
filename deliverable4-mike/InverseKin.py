from functions import *

# starting position
print('Moving to Initial Position')

#zero position for both arms
r=0
rpose = rightArmPose(0,r,r,r,r,r,r,r)   
moveRight(np.zeros(8))
l=0
lpose = leftArmPose(45,12,12,12,12,12,12,12)
larray = np.array([45,12,12,12,12,12,12,12])
moveLeft(larray)

moveObj(rpose, clientID, objHandleRightTheoretical)
placeLeftJoints(larray)
moveObj(lpose, clientID, objHandleLeftTheoretical)
detectCollisionLeft(larray)

endSim()

    
















