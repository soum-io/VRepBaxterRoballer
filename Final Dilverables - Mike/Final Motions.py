from functions import *

# starting position
print('Moving to Initial Position')
rarray = np.array([0,0,0,0,0,0,0,0])
larray = np.array([0,0,0,0,0,0,0,0])
totalLeft(larray)
totalRight(rarray)

fireArr = np.array([[1,0,0,-5],
              [0,1,0,-5],
              [0,0,1,-5],
              [0,0,0,1]])
moveObj(fireArr,clientID,fire)


playHenoi()

# =============================================================================
# getLeftToPoint()
# getLeftToPoint()
# LeftCup(True)
# getLeftToPoint()
# getLeftToPoint()
# LeftCup(False)
# getRightToPoint()
# =============================================================================


endSim()

    
















