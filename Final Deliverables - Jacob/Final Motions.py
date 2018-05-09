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


playHenoi(4)

'''
x_start = .7
y_start = 0
x_end = .7
y_end = .3
x_mid = .7
y_mid = -.3
'''
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

    
















