from functions import *

# starting position
print('Moving to Initial Position')

#zero position for both arms

#spin ,check no collisions and outsdie collisons
rot_angle = 0
for i in range(20):
    rot_angle = rot_angle + 90/20
    rarray = np.array([rot_angle,0,0,0,0,0,0,0])
    larray = np.array([rot_angle,0,0,0,0,0,0,0])
    
    totalLeft(larray)
    totalRight(rarray)

for i in range(20):
    rot_angle = rot_angle - 90/20
    rarray = np.array([rot_angle,0,0,0,0,0,0,0])
    larray = np.array([rot_angle,0,0,0,0,0,0,0])
    
    totalLeft(larray)
    totalRight(rarray)


endSim()

    
















