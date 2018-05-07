from functions import *

# starting position - move baxter's joints to 0 degrees
print('Moving to Initial Position')
rarray = np.array([0,0,0,0,0,0,0,0])
larray = np.array([0,0,0,0,0,0,0,0])
totalLeft(larray)
totalRight(rarray)


#this is the fire object that will be displayed
#when the puzzle is solved. Move it out of the way
#for now
fireArr = np.array([[1,0,0,-5],
              [0,1,0,-5],
              [0,0,1,-5],
              [0,0,0,1]])
moveObj(fireArr,clientID,fire)

#call the function to play Tower of Hanoi
playHanoi()


endSim()

    
















