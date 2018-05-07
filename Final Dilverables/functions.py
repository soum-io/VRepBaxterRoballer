'''
This file contains all the functions that will be used in the final video 
diliverable. It used variables declared in variables.py, and is called in 
Final Motions.py
'''
#import all the variables we will need
from variables import *

#takes a 3x1 matrix, and returns the 3x3 skew
def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

#takes a 3x3 skew matrix and returns the original 3x1 matrix
def dskew(x):
    return np.array[[x[2][1]],[x[0][2]],[x[1][0]]]

#tajes a 6x1 matrix and returns the 6x6 skew
def vskew(x):
    return np.array([[0,-x[2],x[1],x[3]],
                     [x[2],0,-x[0],x[4]],
                     [-x[1],x[0],0,x[5]],
                     [0,0,0,0]])  
    
#takes a 6x6 skew matrix and reutrns a 6x1 skew
def dvskew(x):
    return np.array([[x[2][1]],
                    [x[0][2]],
                    [x[1][0]],
                    [x[0][3]],
                    [x[1][3]],
                    [x[2][3]]])
    

#Makes the jacobian matrix for inverse kinmatics of a robot. 
#Inputs are the set of thetas for the joints of the robot and 
#and the skrews axis' of each joint
def Jmaker(theta,s):
    joints = s.shape[1];
    temp = np.zeros((6,joints))
    for i in range(joints):
        if(i == 0):
            temp[:,0] = s[:,0].reshape((6))
            continue 
        cur = sl.expm(vskew(s[:,0])*theta[0])
        for y in range(1,i):
            cur = cur.dot(sl.expm(vskew(s[:,y])*theta[y]))
        temp[:,i] = admaker(cur).dot(s[:,i])
    return temp

#returns the matrix exponential of the skew of a screw times its respective
#theta. Used as a helper, not helpul by itself.
def exp(s,t):
    return sl.expm(vskew(s)*t)
    
#return the adjoint matrix of a 3x3 rotation matrix
def admaker(x):
    temp = np.zeros((6,6))
    temp[:3,:3] = np.copy(x[:3,:3])
    temp[3:,:3] = np.dot(skew(x[:3,3]), x[:3,:3])
    temp[3:,3:] = np.copy(x[:3,:3])
    return temp

#convert to radians
def rad(x):
    return math.radians(x)

#return sine of angle in degrees
def sind(x):
    return np.sin(rad(x))

#return cosine of angle in degrees
def cosd(x):
    return np.cos(rad(x))

#Return the final pose of a arm given the starting Transormation Matrix T,
#the set of thetas for each joint, and the skrew axis' for each joint.
#This is forward kinematics
def TtoM(theta, s,M):
    T = np.identity(4)
    for i in range(theta.size):
        T = T.dot(sl.expm(vskew(s[:,i])*theta[i,0]))
    return T.dot(M)

#used in inver kinematics. Return the theta dot given the current Jacobian and 
#skrew
def td(J, V):
    return nl.inv(np.transpose(J)@J+.1*np.identity(J.shape[1]))@np.transpose(J)@V


#Function used to move around Vrep objects given the Desired Transofrmation 
#matrix and the object handle defined in variables.py
def moveObj(T, clientID, objHandle):    
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    cy_thresh = 0
    _FLOAT_EPS_4 = np.finfo(float).eps * 4.0
    try:
        cy_thresh = np.finfo(R.dtype).eps * 4
    except ValueError:
        cy_thresh = _FLOAT_EPS_4 
    r11, r12, r13, r21, r22, r23, r31, r32, r33 = R.flat
    cy = math.sqrt(r33*r33 + r23*r23)
    if cy > cy_thresh: # cos(y) not close to zero, standard form
        z = math.atan2(-r12,  r11) # atan2(cos(y)*sin(z), cos(y)*cos(z))
        y = math.atan2(r13,  cy) # atan2(sin(y), cy)
        x = math.atan2(-r23, r33) # atan2(cos(y)*sin(x), cos(x)*cos(y))
    else: # cos(y) (close to) zero, so x -> 0.0 (see above)
        # so r21 -> sin(z), r22 -> cos(z) and
        z = math.atan2(r21,  r22)
        y = math.atan2(r13,  cy) # atan2(sin(y), cy)
        x = 0.0
    E = np.array([x, y, z])
    
    vrep.simxSetObjectPosition(clientID, objHandle, -1, p, vrep.simx_opmode_oneshot)
    vrep.simxSetObjectOrientation(clientID, objHandle, -1, E, vrep.simx_opmode_oneshot)

#returns the pose of Baxter's left arm given a set of thetas for each arm
def leftArmPose(thetal):
    #left arm (facing towards baxter)
    thetaLeft = np.array([[math.radians(thetal[0])], [math.radians(thetal[1])], [math.radians(thetal[2])], [math.radians(thetal[3])], [math.radians(thetal[4])], [math.radians(thetal[5])], [math.radians(thetal[6])], [math.radians(thetal[7])]])
        
    
    #defining all of the skrew axis for each joint
    a0 = np.array([[0],[0],[1]])
    q0 = np.array([[.0177],[.0032],[.8777]])
    bottom0 = np.dot(-skew(a0),q0)
    S0 = np.zeros((6,1))
    S0[:3] = np.copy(a0)
    S0[3:] = np.copy(bottom0)
        
    a1 = np.array([[0],[0],[1]])
    q1 = np.array([[.0815],[.2622],[1.0540]])
    bottom1 = np.dot(-skew(a1),q1)
    S1 = np.zeros((6,1))
    S1[:3] = np.copy(a1)
    S1[3:] = np.copy(bottom1)
    
    a2 = np.array([[-1],[0],[0]])
    q2 = np.array([[.0815],[.3312],[1.3244]])
    bottom2 = np.dot(-skew(a2),q2)
    S2 = np.zeros((6,1))
    S2[:3] = np.copy(a2)
    S2[3:] = np.copy(bottom2)
    
    a3 = np.array([[0],[1],[0]])
    q3 = np.array([[.0815],[.4332],[1.3244]])
    bottom3 = np.dot(-skew(a3),q3)
    S3 = np.zeros((6,1))
    S3[:3] = np.copy(a3)
    S3[3:] = np.copy(bottom3)
    
    a4 = np.array([[-1],[0],[0]])
    q4 = np.array([[.0815],[.6956],[1.2554]])
    bottom4 = np.dot(-skew(a4),q4)
    S4 = np.zeros((6,1))
    S4[:3] = np.copy(a4)
    S4[3:] = np.copy(bottom4)
    
    a5 = np.array([[0],[1],[0]])
    q5 = np.array([[.0815],[.7992],[1.2554]])
    bottom5 = np.dot(-skew(a5),q5)
    S5 = np.zeros((6,1))
    S5[:3] = np.copy(a5)
    S5[3:] = np.copy(bottom5)
    
    a6 = np.array([[-1],[0],[0]])
    q6 = np.array([[.0815],[1.0699],[1.2454]])
    bottom6 = np.dot(-skew(a6),q6)
    S6 = np.zeros((6,1))
    S6[:3] = np.copy(a6)
    S6[3:] = np.copy(bottom6)
    
    a7 = np.array([[0],[1],[0]])
    q7 = np.array([[.0815],[1.1859],[1.2454]])
    bottom7 = np.dot(-skew(a7),q7)
    S7 = np.zeros((6,1))
    S7[:3] = np.copy(a7)
    S7[3:] = np.copy(bottom7)
    
    s = np.zeros((6,thetaLeft.size))
    s[:,0] = S0.reshape((6))
    s[:,1] = S1.reshape((6))
    s[:,2] = S2.reshape((6))
    s[:,3] = S3.reshape((6))
    s[:,4] = S4.reshape((6))
    s[:,5] = S5.reshape((6))
    s[:,6] = S6.reshape((6))
    s[:,7] = S7.reshape((6))
    
    #ML defined in variables as starting Left arm pose
    finalLeft = TtoM(thetaLeft,s,ML )
    return finalLeft

#returns the pose of Baxter's right arm given a set of thetas for each arm
def rightArmPose(thetar):
    #right arm
    thetaRight= np.array([[math.radians(thetar[0])], [math.radians(thetar[1])], [math.radians(thetar[2])], [math.radians(thetar[3])], [math.radians(thetar[4])], [math.radians(thetar[5])], [math.radians(thetar[6])], [math.radians(thetar[7])]])
     
    
    #define skrew axis for right arm joints
    a0 = np.array([[0],[0],[1]])
    q0 = np.array([[.0177],[.0032],[.8777]])
    bottom0 = np.dot(-skew(a0),q0)
    S0 = np.zeros((6,1))
    S0[:3] = np.copy(a0)
    S0[3:] = np.copy(bottom0)
        
    a1 = np.array([[0],[0],[1]])
    q1 = np.array([[.0818],[-.2558],[1.0540]])
    bottom1 = np.dot(-skew(a1),q1)
    S1 = np.zeros((6,1))
    S1[:3] = np.copy(a1)
    S1[3:] = np.copy(bottom1)
    
    a2 = np.array([[1],[0],[0]])
    q2 = np.array([[.0818],[-.3248],[1.3244]])
    bottom2 = np.dot(-skew(a2),q2)
    S2 = np.zeros((6,1))
    S2[:3] = np.copy(a2)
    S2[3:] = np.copy(bottom2)
    
    a3 = np.array([[0],[-1],[0]])
    q3 = np.array([[.0818],[-.4268],[1.3244]])
    bottom3 = np.dot(-skew(a3),q3)
    S3 = np.zeros((6,1))
    S3[:3] = np.copy(a3)
    S3[3:] = np.copy(bottom3)
    
    a4 = np.array([[1],[0],[0]])
    q4 = np.array([[.0818],[-.6892],[1.2554]])
    bottom4 = np.dot(-skew(a4),q4)
    S4 = np.zeros((6,1))
    S4[:3] = np.copy(a4)
    S4[3:] = np.copy(bottom4)
    
    a5 = np.array([[0],[-1],[0]])
    q5 = np.array([[.0818],[-.7928],[1.2554]])
    bottom5 = np.dot(-skew(a5),q5)
    S5 = np.zeros((6,1))
    S5[:3] = np.copy(a5)
    S5[3:] = np.copy(bottom5)
    
    a6 = np.array([[1],[0],[0]])
    q6 = np.array([[.0818],[-1.0635],[1.2454]])
    bottom6 = np.dot(-skew(a6),q6)
    S6 = np.zeros((6,1))
    S6[:3] = np.copy(a6)
    S6[3:] = np.copy(bottom6)
    
    a7 = np.array([[0],[-1],[0]])
    q7 = np.array([[.0818],[-1.1795],[1.2454]])
    bottom7 = np.dot(-skew(a7),q7)
    S7 = np.zeros((6,1))
    S7[:3] = np.copy(a7)
    S7[3:] = np.copy(bottom7)
    
    s = np.zeros((6,thetaRight.size))
    s[:,0] = S0.reshape((6))
    s[:,1] = S1.reshape((6))
    s[:,2] = S2.reshape((6))
    s[:,3] = S3.reshape((6))
    s[:,4] = S4.reshape((6))
    s[:,5] = S5.reshape((6))
    s[:,6] = S6.reshape((6))
    s[:,7] = S7.reshape((6))
    
    #MR defined in variables as starting pose for right arm
    finalRight = TtoM(thetaRight,s,MR )
    return finalRight


#returns the initial left arm tip pose with all thetas set to 0;
def leftArmS():
    a0 = np.array([[0],[0],[1]])
    q0 = np.array([[.0177],[.0032],[.8777]])
    bottom0 = np.dot(-skew(a0),q0)
    S0 = np.zeros((6,1))
    S0[:3] = np.copy(a0)
    S0[3:] = np.copy(bottom0)
        
    a1 = np.array([[0],[0],[1]])
    q1 = np.array([[.0815],[.2622],[1.0540]])
    bottom1 = np.dot(-skew(a1),q1)
    S1 = np.zeros((6,1))
    S1[:3] = np.copy(a1)
    S1[3:] = np.copy(bottom1)
    
    a2 = np.array([[-1],[0],[0]])
    q2 = np.array([[.0815],[.3312],[1.3244]])
    bottom2 = np.dot(-skew(a2),q2)
    S2 = np.zeros((6,1))
    S2[:3] = np.copy(a2)
    S2[3:] = np.copy(bottom2)
    
    a3 = np.array([[0],[1],[0]])
    q3 = np.array([[.0815],[.4332],[1.3244]])
    bottom3 = np.dot(-skew(a3),q3)
    S3 = np.zeros((6,1))
    S3[:3] = np.copy(a3)
    S3[3:] = np.copy(bottom3)
    
    a4 = np.array([[-1],[0],[0]])
    q4 = np.array([[.0815],[.6956],[1.2554]])
    bottom4 = np.dot(-skew(a4),q4)
    S4 = np.zeros((6,1))
    S4[:3] = np.copy(a4)
    S4[3:] = np.copy(bottom4)
    
    a5 = np.array([[0],[1],[0]])
    q5 = np.array([[.0815],[.7992],[1.2554]])
    bottom5 = np.dot(-skew(a5),q5)
    S5 = np.zeros((6,1))
    S5[:3] = np.copy(a5)
    S5[3:] = np.copy(bottom5)
    
    a6 = np.array([[-1],[0],[0]])
    q6 = np.array([[.0815],[1.0699],[1.2454]])
    bottom6 = np.dot(-skew(a6),q6)
    S6 = np.zeros((6,1))
    S6[:3] = np.copy(a6)
    S6[3:] = np.copy(bottom6)
    
    a7 = np.array([[0],[1],[0]])
    q7 = np.array([[.0815],[1.1859],[1.2454]])
    bottom7 = np.dot(-skew(a7),q7)
    S7 = np.zeros((6,1))
    S7[:3] = np.copy(a7)
    S7[3:] = np.copy(bottom7)
    
    s = np.zeros((6,8))
    s[:,0] = S0.reshape((6))
    s[:,1] = S1.reshape((6))
    s[:,2] = S2.reshape((6))
    s[:,3] = S3.reshape((6))
    s[:,4] = S4.reshape((6))
    s[:,5] = S5.reshape((6))
    s[:,6] = S6.reshape((6))
    s[:,7] = S7.reshape((6))
    
    return s

#returns the intial right arm tip pose with all thetas set to 0
def rightArmS():
        
    a0 = np.array([[0],[0],[1]])
    q0 = np.array([[.0177],[.0032],[.8777]])
    bottom0 = np.dot(-skew(a0),q0)
    S0 = np.zeros((6,1))
    S0[:3] = np.copy(a0)
    S0[3:] = np.copy(bottom0)
        
    a1 = np.array([[0],[0],[1]])
    q1 = np.array([[.0818],[-.2558],[1.0540]])
    bottom1 = np.dot(-skew(a1),q1)
    S1 = np.zeros((6,1))
    S1[:3] = np.copy(a1)
    S1[3:] = np.copy(bottom1)
    
    a2 = np.array([[1],[0],[0]])
    q2 = np.array([[.0818],[-.3248],[1.3244]])
    bottom2 = np.dot(-skew(a2),q2)
    S2 = np.zeros((6,1))
    S2[:3] = np.copy(a2)
    S2[3:] = np.copy(bottom2)
    
    a3 = np.array([[0],[-1],[0]])
    q3 = np.array([[.0818],[-.4268],[1.3244]])
    bottom3 = np.dot(-skew(a3),q3)
    S3 = np.zeros((6,1))
    S3[:3] = np.copy(a3)
    S3[3:] = np.copy(bottom3)
    
    a4 = np.array([[1],[0],[0]])
    q4 = np.array([[.0818],[-.6892],[1.2554]])
    bottom4 = np.dot(-skew(a4),q4)
    S4 = np.zeros((6,1))
    S4[:3] = np.copy(a4)
    S4[3:] = np.copy(bottom4)
    
    a5 = np.array([[0],[-1],[0]])
    q5 = np.array([[.0818],[-.7928],[1.2554]])
    bottom5 = np.dot(-skew(a5),q5)
    S5 = np.zeros((6,1))
    S5[:3] = np.copy(a5)
    S5[3:] = np.copy(bottom5)
    
    a6 = np.array([[1],[0],[0]])
    q6 = np.array([[.0818],[-1.0635],[1.2454]])
    bottom6 = np.dot(-skew(a6),q6)
    S6 = np.zeros((6,1))
    S6[:3] = np.copy(a6)
    S6[3:] = np.copy(bottom6)
    
    a7 = np.array([[0],[-1],[0]])
    q7 = np.array([[.0818],[-1.1795],[1.2454]])
    bottom7 = np.dot(-skew(a7),q7)
    S7 = np.zeros((6,1))
    S7[:3] = np.copy(a7)
    S7[3:] = np.copy(bottom7)
    
    s = np.zeros((6,8))
    s[:,0] = S0.reshape((6))
    s[:,1] = S1.reshape((6))
    s[:,2] = S2.reshape((6))
    s[:,3] = S3.reshape((6))
    s[:,4] = S4.reshape((6))
    s[:,5] = S5.reshape((6))
    s[:,6] = S6.reshape((6))
    s[:,7] = S7.reshape((6))
    
    return s


#checks if baxter can use a set of thetas given limits angles on each joint.
def check(theta, limits):
    count = 0
    for ele in theta:
        if(count == 0):
            count = count + 1
            continue
        if(ele >= limits[count][0] and ele <= limits[count][1]):
            count = count + 1
            continue
        else:
            return False
    return True


#return the thetas that will allow Baxters left arm reach desired position
def invLeftArm(xl, yl, zl):
    #start at zero position

    leftTip = int(vrep.simxGetObjectHandle(clientID, 'Baxter_leftArm_tip', vrep.simx_opmode_blocking)[1])
    T_2in0 = np.array([[0, 0, 1, xl], [-1, 0, 0, yl], [0, -1, 0, zl], [0.00000000, 0.00000000, 0.00000000, 1.00000000]])
    moveObj(T_2in0, clientID, objHandleLeftTheoretical)
        
    SL = leftArmS()
    thetaL = np.zeros((SL.shape[1],1))
    thetadot = np.array([100])
    
    TL_1in0 = ML
    J = Jmaker(thetaL,SL)
    V0 = dvskew(sl.logm(T_2in0.dot(nl.inv(TL_1in0))))
    V = nl.inv(admaker(TL_1in0))@V0
    while(True):
        failCount = 0;
        count = 0
        while nl.norm(V) >=.01 and nl.norm(thetadot) >= .0001:
            thetadot = td(J,V0)
            thetaL = thetaL + thetadot*1
            TL_1in0 = TtoM(thetaL,SL,ML)
            J = Jmaker(thetaL, SL)
            V0 = dvskew(sl.logm(T_2in0.dot(nl.inv(TL_1in0))))
            V = nl.inv(admaker(TL_1in0))@V0
            count = count + 1
            if(count == 60):
                print("Norm of V could not merge - Using new starting position and trying again")
                print()
                thetaL = np.array([[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)]])
                thetadot = np.array([100])
                TL_1in0 = TtoM(thetaL,SL,ML)
                J = Jmaker(thetaL,SL)
                V0 = dvskew(sl.logm(T_2in0.dot(nl.inv(TL_1in0))))
                V = dvskew(sl.logm(T_2in0.dot(nl.inv(TL_1in0))))
                count = 0
                #checking for how many times this happens. So we can detect if a position is unreachable
                failCount = failCount + 1
                if(failCount >= 70):
                    print("Position Unreachable")
                    print()
                    return None
        #checking if the angles are within limit of their respective joints
        for i in range(thetaL.size):
            if(thetaL[i,0] > math.pi):
                thetaL[i,0] = thetaL[i,0] - 2*math.pi
            elif(thetaL[i,0] < -math.pi):
                thetaL[i,0] = thetaL[i,0] + 2*math.pi
        if(check(thetaL, LeftLimits)):
            break
        else:
            print("Came up with set of thetas where some of them were out of range, trying again with different starting position")
            print()
            #try different starting orientation to get different end angles that will hopefully meet the angle limits of the joints
            thetaL = np.array([[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)]])
            thetadot = np.array([100])
            TL_1in0 = TtoM(thetaL,SL,ML)
            J = Jmaker(thetaL,SL)
            V0 = dvskew(sl.logm(T_2in0.dot(nl.inv(TL_1in0))))
            V = dvskew(sl.logm(T_2in0.dot(nl.inv(TL_1in0))))
    return thetaL
        
#return the thetas that will allow Baxters right arm reach desired position     
def invRightArm(xr, yr, zr):
    #start at zero position    

    rightTip = int(vrep.simxGetObjectHandle(clientID, 'Baxter_rightArm_tip', vrep.simx_opmode_blocking)[1])
    T_2in0 = np.array([[0, 0, 1, xr], [1, 0, 0, yr], [0, 1, 0, zr], [0.00000000, 0.00000000, 0.00000000, 1.00000000]])
    moveObj(T_2in0, clientID, objHandleRightTheoretical)
        
    SR = rightArmS()
    thetaR = np.zeros((SR.shape[1],1))
    thetadot = np.array([100])
    
    TR_1in0 = MR
    J = Jmaker(thetaR,SR)
    V0 = dvskew(sl.logm(T_2in0.dot(nl.inv(TR_1in0))))
    V = nl.inv(admaker(TR_1in0))@V0
    while(True):
        count = 0
        failCount = 0
        while nl.norm(V) >=.01 and nl.norm(thetadot) >= .0001:
            thetadot = td(J,V0)
            thetaR = thetaR + thetadot*1
            TR_1in0 = TtoM(thetaR,SR,MR)
            J = Jmaker(thetaR, SR)
            V0 = dvskew(sl.logm(T_2in0.dot(nl.inv(TR_1in0))))
            V = nl.inv(admaker(TR_1in0))@V0
            count = count + 1
            if(count == 60):
                
                print("Norm of V could not merge - Using new starting position and trying again")
                print()
                thetaR = np.array([[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)]])
                thetadot = np.array([100])
                TR_1in0 = TtoM(thetaR,SR,MR)
                J = Jmaker(thetaR,SR)
                V0 = dvskew(sl.logm(T_2in0.dot(nl.inv(TR_1in0))))
                V = dvskew(sl.logm(T_2in0.dot(nl.inv(TR_1in0))))
                count = 0
                failCount = failCount + 1
                if(failCount >= 70):
                    print("Position Unreachable")
                    print()
                    return None
        #checking if the angles are within limit of their respective joints
        for i in range(thetaR.size):
            if(thetaR[i,0] > math.pi):
                thetaR[i,0] = thetaR[i,0] - 2*math.pi
            elif(thetaR[i,0] < -math.pi):
                thetaR[i,0] = thetaR[i,0] + 2*math.pi
        if(check(thetaR, rightLimits)):
            break
        else:
            
            print("Came up with set of thetas where some of them were out of range, trying again with different starting position")
            print()
            #try different starting orientation to get different end angles 
            thetaR = np.array([[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)]])
            thetadot = np.array([100])
            TR_1in0 = TtoM(thetaR,SR,MR)
            J = Jmaker(thetaR,SR)
            V0 = dvskew(sl.logm(T_2in0.dot(nl.inv(TR_1in0))))
            V = dvskew(sl.logm(T_2in0.dot(nl.inv(TR_1in0)))) 
    return thetaR
    
#in the vrep simulation, move Baxter's left arm joints to desired thetas
def moveLeft(thetas):
    vrep.simxSetJointTargetPosition(clientID, rotJoint, math.radians(thetas[0]), vrep.simx_opmode_oneshot) 
    for i in range(0,7):
        vrep.simxSetJointTargetPosition(clientID, leftArm[i], math.radians(thetas[i+1]), vrep.simx_opmode_oneshot) 
   
#in the vrep simulation, move Baxter's right arm joints to desired thetas     
def moveRight(thetas):
    vrep.simxSetJointTargetPosition(clientID, rotJoint, math.radians(thetas[0]), vrep.simx_opmode_oneshot) 
    for i in range(0,7):
        vrep.simxSetJointTargetPosition(clientID, rightArm[i], math.radians(thetas[i+1]), vrep.simx_opmode_oneshot) 

#end any current playing simulation
def endSim():
    input("Press Enter to end the simulation.")
    # Stop simulation
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
    
    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)
    
    # Close the connection to V-REP
    vrep.simxFinish(clientID)


#used for collision detection. Move dummy variables at each joint for baxter left
#arm. The dummy objects are spheres that act as the surface area for that part 
#of the arm
def placeLeftJoints(theta):
    thetal = np.zeros(theta.size)
    for i in range(thetal.size):
        thetal[i] = math.radians(theta[i])
        
    S = leftArmS()
    M = ML
    coords = np.zeros((3,8))
    
    coords[:,0] = np.array([[.0177],[.0032],[.8777]]).reshape((3))
    coords[:,1] = np.array([[.0815],[.2622],[1.0540]]).reshape((3))
    coords[:,2] = np.array([[.0815],[.3312],[1.3244]]).reshape((3))
    coords[:,3] = np.array([[.0815],[.4332],[1.3244]]).reshape((3))
    coords[:,4] = np.array([[.0815],[.6956],[1.2554]]).reshape((3))
    coords[:,5] = np.array([[.0815],[.7992],[1.2554]]).reshape((3))
    coords[:,6] = np.array([[.0815],[1.0699],[1.2454]]).reshape((3))
    coords[:,7] = np.array([[.0815],[1.1859],[1.2454]]).reshape((3))
    
    rotBaseM = np.copy(M)
    rotBaseM[:3, 3] = np.array([[.0177],[.0032],[.8777]]).reshape((3))
    moveObj(rotBaseM, clientID, leftArmDummies[0])
    
    
    for i in range(thetal.size):
        startM = np.copy(M)
        if(i != S[0].size-1):
            startM[0][3] = coords[0][i+1]
            startM[1][3] = coords[1][i+1]
            startM[2][3] = coords[2][i+1]
        Mt = TtoM(thetal[:i+1].reshape((i+1,1)), S[:,:i+1], startM)
        moveObj(Mt, clientID, leftArmDummies[i+1])
     
        
#used for collision detection. Move dummy variables at each joint for baxter's right
#arm. The dummy objects are spheres that act as the surface area for that part 
#of the arm
def placeRightJoints(theta):
    thetal = np.zeros(theta.size)
    for i in range(thetal.size):
        thetal[i] = math.radians(theta[i])
        
    S = rightArmS()
    M = MR
    coords = np.zeros((3,8))
    
    coords[:,0] = np.array([[.0177],[.0032],[.8777]]).reshape((3))
    coords[:,1] = np.array([[.0818],[-.2558],[1.0540]]).reshape((3))
    coords[:,2] = np.array([[.0818],[-.3248],[1.3244]]).reshape((3))
    coords[:,3] = np.array([[.0818],[-.4268],[1.3244]]).reshape((3))
    coords[:,4] = np.array([[.0818],[-.6892],[1.2554]]).reshape((3))
    coords[:,5] = np.array([[.0818],[-.7928],[1.2554]]).reshape((3))
    coords[:,6] = np.array([[.0818],[-1.0635],[1.2454]]).reshape((3))
    coords[:,7] = np.array([[.0818],[-1.1795],[1.2454]]).reshape((3))

    
    rotBaseM = np.copy(M)
    rotBaseM[:3, 3] = np.array([[.0177],[.0032],[.8777]]).reshape((3))
    moveObj(rotBaseM, clientID, rightArmDummies[0])
    
    
    for i in range(thetal.size):
        startM = np.copy(M)
        if(i != S[0].size-1):
            startM[0][3] = coords[0][i+1]
            startM[1][3] = coords[1][i+1]
            startM[2][3] = coords[2][i+1]
        Mt = TtoM(thetal[:i+1].reshape((i+1,1)), S[:,:i+1], startM)
        moveObj(Mt, clientID, rightArmDummies[i+1])
                    
        
#detect if a given set of theta for Baxter's left arm results in a collision 
#with self of outside objects defined in variables.py. Two spheres are in 
#in collision if their distance is less that the addition of their radius'
def detectCollisionLeft(thetal):
    
    theta = np.zeros(thetal.size)
    for i in range(thetal.size):
        theta[i] = math.radians(thetal[i])
    col = False
    
    if(not check(theta,LeftLimits)):
        print("Left: Joint angles out of range")
        return True
    
    S = leftArmS()
    M = ML
    r_robot = np.array([[0,.2,.1,.05,.05,.05,.05,.05,.05,.05]])

    
    coords = np.zeros((3,8))   
    coords[:,0] = np.array([[.0177],[.0032],[.8777]]).reshape((3))
    coords[:,1] = np.array([[.0815],[.2622],[1.0540]]).reshape((3))
    coords[:,2] = np.array([[.0815],[.3312],[1.3244]]).reshape((3))
    coords[:,3] = np.array([[.0815],[.4332],[1.3244]]).reshape((3))
    coords[:,4] = np.array([[.0815],[.6956],[1.2554]]).reshape((3))
    coords[:,5] = np.array([[.0815],[.7992],[1.2554]]).reshape((3))
    coords[:,6] = np.array([[.0815],[1.0699],[1.2454]]).reshape((3))
    coords[:,7] = np.array([[.0815],[1.1859],[1.2454]]).reshape((3))
            
    
    r = np.zeros((1,r_robot.size+r_obstacle.size))
    r[0,:r_robot.size] = np.copy(r_robot)
    r[0,r_robot.size:] = np.copy(r_obstacle)
    
    centers = np.zeros((3,r.size))
    
    centers[0][1] = coords[0][0]
    centers[1][1] = coords[1][0]
    centers[2][1] = coords[2][0]
    
    centers[:,r_robot.size:] = np.copy(p_obstacle)
    
    
    
    for i in range(theta.size):
        startM = np.copy(M)
        if(i != S[0].size-1):
            startM[0][3] = coords[0][i+1]
            startM[1][3] = coords[1][i+1]
            startM[2][3] = coords[2][i+1]
        Mt = TtoM(theta[:i+1].reshape((i+1,1)), S[:,:i+1], startM)
        
        centers[0][i+2] = Mt[0][3]
        centers[1][i+2] = Mt[1][3]
        centers[2][i+2] = Mt[2][3] 
    for l in range(9):
        for y in range(l+1,r.size):
            dist = nl.norm(centers[:,l] - centers[:,y])
            if dist < r[0][l]+r[0][y]:
                if y >= r_robot[0].size or l >= r_robot[0].size:
                    print("Left: collision with outside object")
                    col = True
                    return col
                else:
                    print("Left: collision with self")
                    col = True
                    return col

    return col

#detect if a given set of theta for Baxter's right arm results in a collision 
#with self of outside objects defined in variables.py. Two spheres are in 
#in collision if their distance is less that the addition of their radius'
def detectCollisionRight(thetal):
    theta = np.zeros(thetal.size)
    for i in range(thetal.size):
        theta[i] = math.radians(thetal[i])
    col = False
    
    if(not check(theta,rightLimits)):
        print("Right: Joint angles out of range")
        return True

    S = rightArmS()
    M = MR
    r_robot = np.array([[0,.2,.1,.05,.05,.05,.05,.05,.05,.05]])

    
    coords = np.zeros((3,8))   
    coords[:,0] = np.array([[.0177],[.0032],[.8777]]).reshape((3))
    coords[:,1] = np.array([[.0818],[-.2558],[1.0540]]).reshape((3))
    coords[:,2] = np.array([[.0818],[-.3248],[1.3244]]).reshape((3))
    coords[:,3] = np.array([[.0818],[-.4268],[1.3244]]).reshape((3))
    coords[:,4] = np.array([[.0818],[-.6892],[1.2554]]).reshape((3))
    coords[:,5] = np.array([[.0818],[-.7928],[1.2554]]).reshape((3))
    coords[:,6] = np.array([[.0818],[-1.0635],[1.2454]]).reshape((3))
    coords[:,7] = np.array([[.0818],[-1.1795],[1.2454]]).reshape((3))
            
    
    r = np.zeros((1,r_robot.size+r_obstacle.size))
    r[0,:r_robot.size] = np.copy(r_robot)
    r[0,r_robot.size:] = np.copy(r_obstacle)
    
    centers = np.zeros((3,r.size))
    
    centers[0][1] = coords[0][0]
    centers[1][1] = coords[1][0]
    centers[2][1] = coords[2][0]
    
    centers[:,r_robot.size:] = np.copy(p_obstacle)
    
    
    
    for i in range(theta.size):
        startM = np.copy(M)
        if(i != S[0].size-1):
            startM[0][3] = coords[0][i+1]
            startM[1][3] = coords[1][i+1]
            startM[2][3] = coords[2][i+1]
        Mt = TtoM(theta[:i+1].reshape((i+1,1)), S[:,:i+1], startM)
        
        centers[0][i+2] = Mt[0][3]
        centers[1][i+2] = Mt[1][3]
        centers[2][i+2] = Mt[2][3] 
    for l in range(9):
        for y in range(l+1,r.size):
            dist = nl.norm(centers[:,l] - centers[:,y])
            if dist < r[0][l]+r[0][y]:
                if y >= r_robot[0].size or l >= r_robot[0].size:
                    print("Right: collision with outside object")
                    col = True
                    return col
                else:
                    print("Right: collision with self")
                    col = True
                    return col
    return col

#calls all functions to move baxter's left arm, and collision spheres for each joint
#to a specific theta set
def totalLeft(larray):
    lpose = leftArmPose(larray)
    moveLeft(larray)
    placeLeftJoints(larray)
    moveObj(lpose, clientID, objHandleLeftTheoretical)
    detectCollisionLeft(larray)
    
#calls all functions to move baxter's right arm, and collision spheres for each joint
#to a specific theta set
def totalRight(rarray):
    rpose = rightArmPose(rarray)
    moveRight(rarray)
    placeRightJoints(rarray)
    moveObj(rpose, clientID, objHandleRightTheoretical)
    detectCollisionRight(rarray)
    
#detects if baxter is in collision with himself or any outside objects, given
# radius of spheres that reperesent each of his joints, the position of obstacles
#and their respective radius, and the theta configuration of baxter
def col_det(S, M, coords, r_robot, p_obstacle, r_obstacle, theta):
    r = np.zeros(r_robot.size+r_obstacle.size)
    r[:r_robot.size] = np.copy(r_robot)
    r[r_robot.size:] = np.copy(r_obstacle)
    
    
    centers = np.zeros((3,r.size))
    centers[0][1] = coords[0][0]
    centers[1][1] = coords[1][0]
    centers[2][1] = coords[2][0]
    centers[:,S[0].size+2:] = np.copy(p_obstacle[:,:])
    
    for i in range(S[0].size):
        startM = np.copy(M)
        if(i != S[0].size-1):
            startM[0][3] = coords[0][i+1]
            startM[1][3] = coords[1][i+1]
            startM[2][3] = coords[2][i+1]
        Mt = TtoM(theta[:i+1].reshape((i+1,1)), S[:,:i+1], startM)
        
        centers[0][i+2] = Mt[0][3]
        centers[1][i+2] = Mt[1][3]
        centers[2][i+2] = Mt[2][3] 
    for l in range(S[0].size+2):
        if tableCollision(centers[0,l], centers[1,l], centers[2,l]):
            return True
        for y in range(S[0].size+2,r.size):
            if(l==y):
                continue
            dist = nl.norm(centers[:,l] - centers[:,y])
            if dist < r[y]+r[l]:
                return True
            
    return False

#determine whether a clear line path exist between a start theta configuration
#and an end theta configuration
def lineClear(S, M, coords, r_robot, p_obstacle, r_obstacle, theta_start, theta_end):
    sig = .1
    dist = nl.norm(theta_start-theta_end)
    count = 1+math.ceil(dist/sig)
    s = 0
    for i in range(count):
        s = s + 1/count
        theta = (1-s)*theta_start[:,0]+s*theta_end[:,0]
        if(col_det(S, M, coords, r_robot, p_obstacle, r_obstacle, theta)):
            return False
    return True

#Overall function to get right arm to a specified x,y,z point that the user specifies
#Also takes into account where the robot is currently, as well as collision detection
def getRightToPoint(guess = True, x = 0, y = 0, z = 0, sig = .1):
    theta_start = np.zeros((8,1));
    theta_start[0] = vrep.simxGetJointPosition(clientID, rotJoint, vrep.simx_opmode_streaming)[1]
    for i in range(1,8):
        theta_start[i] = vrep.simxGetJointPosition(clientID, rightArm[i-1], vrep.simx_opmode_streaming)[1]
    if guess:
        x = float(input("Enter the x coordinate of where you want the right arm to go: "))
        y = float(input("Enter the y coordinate of where you want the right arm to go: "))
        z = float(input("Enter the z coordinate of where you want the right arm to go: "))
    rotBaseM = np.copy(MR)
    rotBaseM[0,3] = x
    rotBaseM[1,3] = y
    rotBaseM[2,3] = z
    
    moveObj(rotBaseM, clientID, dummyPos)
    theta_goal = invRightArm(x, y, z)
    print(theta_goal)
    if(theta_goal is None):
        print("Sorry, we could not come up with a path to the point because right arm cannont reach it." )
        return
    
    S = rightArmS()
    M = np.copy(MR)
    r_robot = np.array([[0,.2,.1,.05,.05,.05,.05,.05,.05,.05]])
    coords = np.zeros((3,8))   
    coords[:,0] = np.array([[.0177],[.0032],[.8777]]).reshape((3))
    coords[:,1] = np.array([[.0818],[-.2558],[1.0540]]).reshape((3))
    coords[:,2] = np.array([[.0818],[-.3248],[1.3244]]).reshape((3))
    coords[:,3] = np.array([[.0818],[-.4268],[1.3244]]).reshape((3))
    coords[:,4] = np.array([[.0818],[-.6892],[1.2554]]).reshape((3))
    coords[:,5] = np.array([[.0818],[-.7928],[1.2554]]).reshape((3))
    coords[:,6] = np.array([[.0818],[-1.0635],[1.2454]]).reshape((3))
    coords[:,7] = np.array([[.0818],[-1.1795],[1.2454]]).reshape((3))
    
    class Tree(object):
        def __init__(self):
            self.theta = None
            self.parent = None
            self.start = None
    
    rootStart = Tree()
    rootStart.theta = np.copy(theta_start)
    rootStart.start = True
    
    rootEnd = Tree()
    rootEnd.theta = np.copy(theta_goal)
    rootEnd.start = False
    
    accepted = np.array([rootStart, rootEnd])
    if(lineClear(S, M, coords, r_robot, p_obstacle, r_obstacle, rootStart.theta, rootEnd.theta)):
        answer = np.concatenate((theta_start,theta_goal),axis=1)
    else:
        count = 0
        notFound = True
        while(notFound):
            if(count > 1000):
                print("Sorry, the robot configuration could not be reached after 1000 attempts")
            tempTheta = np.zeros((theta_start[:,0].size,1))
            for i in range(theta_start[:,0].size):
                tempTheta[i][0] = random.uniform(rightLimits[i][0], rightLimits[i][1])
            if(col_det(S, M, coords, r_robot, p_obstacle, r_obstacle, tempTheta)):
                continue
            tempT  = TtoM(tempTheta,S,M)
            currentClosest = accepted[0]
            currentDistance = nl.norm(rootStart.theta - tempTheta)
            for i in range(1,accepted.size):
                dist = nl.norm(accepted[i].theta - tempTheta)
                if dist < currentDistance:
                    currentDistance = dist
                    currentClosest = accepted[i]
            if(lineClear(S, M, coords, r_robot, p_obstacle, r_obstacle, currentClosest.theta, tempTheta)):
                node = Tree()
                node.theta = np.copy(tempTheta)
                node.parent = currentClosest
                node.start = currentClosest.start
                secondClosest = None
                secondClosestDist = np.inf
                for ele in accepted:
                    if ele.start != node.start:
                        tempDist = nl.norm(ele.theta - node.theta)
                        if tempDist < secondClosestDist:
                            secondClosest = ele
                            secondClosestDist = tempDist
                if(lineClear(S, M, coords, r_robot, p_obstacle, r_obstacle, node.theta, secondClosest.theta)):
                            fromStart = node
                            fromEnd = secondClosest
                            if(secondClosest.start):
                                fromStart = secondClosest
                                fromEnd = node
                            answer = np.concatenate((fromStart.theta, fromEnd.theta),axis = 1)
                            while(fromStart.parent is not None):
                                fromStart = fromStart.parent
                                answer = np.concatenate((fromStart.theta, answer),axis = 1)
                            while(fromEnd.parent is not None):
                                fromEnd = fromEnd.parent
                                answer = np.concatenate((answer, fromEnd.theta),axis = 1)
                            notFound = False
                            break
                accepted = np.append(accepted, copy.copy(node))
            count = count + 1
    print(repr(answer))
    
    #make left arm go straight up so it doesnt hit anything lol
    rotAngle = math.degrees(theta_start[0])
    larray = np.array([rotAngle,0,-90,0,0,0,0,0])
    totalLeft(larray)
    for x in range(answer[0].size-1):
        theta_start_t = answer[:,x].reshape((8,1))
        theta_end_t = answer[:,x+1].reshape((8,1))
        dist = nl.norm(theta_start_t-theta_end_t)
        count = 1+math.ceil(dist/sig)
        s = 0
        for blagh in range(count+1):
            thetaNext = (1-s)*theta_start_t[:,0]+s*theta_end_t[:,0]
            for p in range(thetaNext.size):
                thetaNext[p] =  math.degrees(thetaNext[p])
            rotAngle = thetaNext[0]
            larray = np.array([rotAngle,0,-90,0,0,0,0,0])
            totalLeft(larray)
            totalRight(thetaNext)
            time.sleep(.05)
            s = s + 1/count
            
            
#Overall function to get left arm to a specified x,y,z point that the user specifies
#Also takes into account where the robot is currently, as well as collision detection
def getLeftToPoint(guess = True, x = 0, y = 0, z = 0, sig = .1):
    theta_start = np.zeros((8,1))
    theta_start[0] = vrep.simxGetJointPosition(clientID, rotJoint, vrep.simx_opmode_streaming)[1]
    for i in range(1,8):
        theta_start[i] = vrep.simxGetJointPosition(clientID, leftArm[i-1], vrep.simx_opmode_streaming)[1]
    if guess:
        x = float(input("Enter the x coordinate of where you want the left arm to go: "))
        y = float(input("Enter the y coordinate of where you want the left arm to go: "))
        z = float(input("Enter the z coordinate of where you want the left arm to go: "))
    rotBaseM = np.copy(ML)
    rotBaseM[0,3] = x
    rotBaseM[1,3] = y
    rotBaseM[2,3] = z
    
    moveObj(rotBaseM, clientID, dummyPos)
    theta_goal = invLeftArm(x, y, z)
    print(theta_goal)
    if(theta_goal is None):
        print("Sorry, we could not come up with a path to the point because left arm cannont reach it." )
        return
    
    S = leftArmS()
    M = np.copy(ML)
    r_robot = np.array([[0,.2,.1,.05,.05,.05,.05,.05,.05,.05]])
    coords = np.zeros((3,8))   
    coords[:,0] = np.array([[.0177],[.0032],[.8777]]).reshape((3))
    coords[:,1] = np.array([[.0815],[.2622],[1.0540]]).reshape((3))
    coords[:,2] = np.array([[.0815],[.3312],[1.3244]]).reshape((3))
    coords[:,3] = np.array([[.0815],[.4332],[1.3244]]).reshape((3))
    coords[:,4] = np.array([[.0815],[.6956],[1.2554]]).reshape((3))
    coords[:,5] = np.array([[.0815],[.7992],[1.2554]]).reshape((3))
    coords[:,6] = np.array([[.0815],[1.0699],[1.2454]]).reshape((3))
    coords[:,7] = np.array([[.0815],[1.1859],[1.2454]]).reshape((3))
        
    
    class Tree(object):
        def __init__(self):
            self.theta = None
            self.parent = None
            self.start = None
    
    rootStart = Tree()
    rootStart.theta = np.copy(theta_start)
    rootStart.start = True
    
    rootEnd = Tree()
    rootEnd.theta = np.copy(theta_goal)
    rootEnd.start = False
    
    accepted = np.array([rootStart, rootEnd])
    if(lineClear(S, M, coords, r_robot, p_obstacle, r_obstacle, rootStart.theta, rootEnd.theta)):
        answer = np.concatenate((theta_start,theta_goal),axis=1)
    else:
        count = 1
        notFound = True
        while(notFound): 
            if(count >=1000):
                print("Sorry, the robot configuration could not be reached after 1000 attempts")
            tempTheta = np.zeros((theta_start[:,0].size,1))
            for i in range(theta_start[:,0].size):
                tempTheta[i][0] = random.uniform(LeftLimits[i][0], LeftLimits[i][1])
            if(col_det(S, M, coords, r_robot, p_obstacle, r_obstacle, tempTheta)):
                continue
            tempT  = TtoM(tempTheta,S,M)
            currentClosest = accepted[0]
            currentDistance = nl.norm(rootStart.theta - tempTheta)
            for i in range(1,accepted.size):
                dist = nl.norm(accepted[i].theta - tempTheta)
                if dist < currentDistance:
                    currentDistance = dist
                    currentClosest = accepted[i]
            if(lineClear(S, M, coords, r_robot, p_obstacle, r_obstacle, currentClosest.theta, tempTheta)):
                node = Tree()
                node.theta = np.copy(tempTheta)
                node.parent = currentClosest
                node.start = currentClosest.start
                secondClosest = None
                secondClosestDist = np.inf
                for ele in accepted:
                    if ele.start != node.start:
                        tempDist = nl.norm(ele.theta - node.theta)
                        if tempDist < secondClosestDist:
                            secondClosest = ele
                            secondClosestDist = tempDist
                if(lineClear(S, M, coords, r_robot, p_obstacle, r_obstacle, node.theta, secondClosest.theta)):
                            fromStart = node
                            fromEnd = secondClosest
                            if(secondClosest.start):
                                fromStart = secondClosest
                                fromEnd = node
                            answer = np.concatenate((fromStart.theta, fromEnd.theta),axis = 1)
                            while(fromStart.parent is not None):
                                fromStart = fromStart.parent
                                answer = np.concatenate((fromStart.theta, answer),axis = 1)
                            while(fromEnd.parent is not None):
                                fromEnd = fromEnd.parent
                                answer = np.concatenate((answer, fromEnd.theta),axis = 1)
                            notFound = False
                            break
                accepted = np.append(accepted, copy.copy(node))
            count = count +1
    print(repr(answer))

    #make left right go straight up so it doesnt hit anything lol
    rotAngle = math.degrees(theta_start[0])
    rarray = np.array([rotAngle,0,-90,0,0,0,0,0])
    totalRight(rarray)
    for x in range(answer[0].size-1):
        theta_start_t = answer[:,x].reshape((8,1))
        theta_end_t = answer[:,x+1].reshape((8,1))
        dist = nl.norm(theta_start_t-theta_end_t)
        count = 1+math.ceil(dist/sig)
        s = 0
        for blagh in range(count+1):
            thetaNext = (1-s)*theta_start_t[:,0]+s*theta_end_t[:,0]
            for p in range(thetaNext.size):
                thetaNext[p] =  math.degrees(thetaNext[p])
            rotAngle = thetaNext[0]
            rarray = np.array([rotAngle,0,-90,0,0,0,0,0])
            totalRight(rarray)
            totalLeft(thetaNext)
            time.sleep(.05)
            s = s + 1/count
   
#function to help detect where cubes are during tower of Hanoi         
def cubeCol(i, x, y , z):
    # when baxter picks up cube, all the values are set to -1
    if cubeCenters[0,i] == -1 and cubeCenters[1,i] == -1 and cubeCenters[2,i] == -1:
        return False
    if (x<=cubeCenters[0,i]+.05 and x >=cubeCenters[0,i]-.05 and y <= cubeCenters[1,i]+.05 and y >= cubeCenters[1,i]-.05 and z >=cubeCenters[2,i]+.05 and z <= cubeCenters[2,i]-.05):
        return True
    return False
    
    
        
        
        
#check collisions with table or any of the cubes      
def tableCollision(x,y,z):
    if (x<=.9 and x >=.5 and y <= .6 and y >= -.6 and z >=0 and z <= .7):
        return True
    if cubeCol(0,x,y,x):
        return True
    if cubeCol(1,x,y,x):
        return True
    if cubeCol(2,x,y,x):
        return True
    return False


#turn left suction cup on or off        
def LeftCup(on):
    if(on):
        vrep.simxSetIntegerSignal(clientID, leftcup, 1, vrep.simx_opmode_oneshot)
    else:
        vrep.simxSetIntegerSignal(clientID, leftcup, 0, vrep.simx_opmode_oneshot)
  
#turn right suction cup on or off      
def RightCup(on):
    if(on):
        vrep.simxSetIntegerSignal(clientID, rightcup, 1, vrep.simx_opmode_oneshot)
    else:
        vrep.simxSetIntegerSignal(clientID, rightcup, 0, vrep.simx_opmode_oneshot)
            
#play game of hanoi on top of a table with three blocks. User picks the start,
#middle, and end position, and then baxter completes the tasks with both arms        
def playHanoi():
    xstart = float(input("Enter the starting x location (between .5, .9): "))
    ystart = float(input("Enter the starting y location (between -.6, -6): "))
    xend = float(input("Enter the ending x location (between .5, .9): "))
    yend = float(input("Enter the ending y location (between -.6, -6): "))
    xmiddle = float(input("Enter the middle x location (between .5, .9): "))
    ymiddle = float(input("Enter the middle y location (between -.6, -6): "))
    
    
    height = .1
    table = .7
    offset = .013
    top = height*3+table+offset+.2
    first = table+height+offset
    second = table+height*2+offset
    third = table+height*3+offset
    midTravel = .1
    slowTravel = .05
    pad = .01
    
    c0start = np.array([[1,0,0,xstart],
                  [0,1,0,ystart],
                  [0,0,1,table+(height/2)+pad],
                  [0,0,0,1]])
    c1start = np.array([[1,0,0,xstart],
                  [0,1,0,ystart],
                  [0,0,1,table+height + (height/2)+pad],
                  [0,0,0,1]])
    c2start = np.array([[1,0,0,xstart],
                  [0,1,0,ystart],
                  [0,0,1,table+ 2*height + (height/2)+pad],
                  [0,0,0,1]])
    moveObj(c0start, clientID, block0)
    moveObj(c1start, clientID, block1)
    moveObj(c2start, clientID, block2)
    
    smoke0Arr = np.array([[1,0,0,xstart],
              [0,1,0,ystart],
              [0,0,1,table],
              [0,0,0,1]])
    moveObj(smoke0Arr,clientID,smoke0)
    
    smoke1Arr = np.array([[1,0,0,xmiddle],
              [0,1,0,ymiddle],
              [0,0,1,table],
              [0,0,0,1]])
    moveObj(smoke1Arr,clientID,smoke1)
    
    smoke2Arr = np.array([[1,0,0,xend],
              [0,1,0,yend],
              [0,0,1,table],
              [0,0,0,1]])
    moveObj(smoke2Arr,clientID,smoke2)
    
    

    
    #tower of hanoi for 3 blocks. Hardcoded for now - don't hate
    getLeftToPoint(False, xstart, ystart, top, midTravel)
    getLeftToPoint(False, xstart, ystart, third, slowTravel)
    LeftCup(True)
    getLeftToPoint(False, xstart, ystart, top, midTravel)
    getLeftToPoint(False, xend, yend, top, midTravel)
    getLeftToPoint(False, xend, yend, first, slowTravel)
    LeftCup(False)
    getLeftToPoint(False, xend, yend, top, midTravel)
    getLeftToPoint(False, xstart, ystart, top, midTravel)
    getLeftToPoint(False, xstart, ystart, second, slowTravel)
    LeftCup(True)
    getLeftToPoint(False, xstart, ystart, top, midTravel)
    getLeftToPoint(False, xmiddle, ymiddle, top, midTravel)
    getLeftToPoint(False, xmiddle, ymiddle, first, slowTravel)
    LeftCup(False)
    getLeftToPoint(False, xmiddle, ymiddle, top, midTravel)
    getLeftToPoint(False, xend, yend, top, midTravel)
    getLeftToPoint(False, xend, yend, first, slowTravel)
    LeftCup(True)
    getLeftToPoint(False, xend, yend, top, midTravel)
    getLeftToPoint(False, xmiddle, ymiddle, top, midTravel)
    getLeftToPoint(False, xmiddle, ymiddle, second, slowTravel)
    LeftCup(False)
    getLeftToPoint(False, xmiddle, ymiddle, top, midTravel)
    getRightToPoint(False, xstart, ystart, top, midTravel)
    getRightToPoint(False, xstart, ystart, first, slowTravel)
    RightCup(True)
    getRightToPoint(False, xstart, ystart, top, midTravel)
    getRightToPoint(False, xend, yend, top, midTravel)
    getRightToPoint(False, xend, yend, first, slowTravel)
    RightCup(False)
    getRightToPoint(False, xend, yend, top, midTravel)
    getRightToPoint(False, xmiddle, ymiddle, top, midTravel)
    getRightToPoint(False, xmiddle, ymiddle, second, slowTravel)
    RightCup(True)
    getRightToPoint(False, xmiddle, ymiddle, top, midTravel)
    getRightToPoint(False, xstart, ystart, top, midTravel)
    getRightToPoint(False, xstart, ystart, first, slowTravel)
    RightCup(False)
    getRightToPoint(False, xstart, ystart, top, midTravel)
    getRightToPoint(False, xmiddle, ymiddle, top, midTravel)
    getRightToPoint(False, xmiddle, ymiddle, first, slowTravel)
    RightCup(True)
    getRightToPoint(False, xmiddle, ymiddle, top, midTravel)
    getRightToPoint(False, xend, yend, top, midTravel)
    getRightToPoint(False, xend, yend, second, slowTravel)
    RightCup(False)
    getRightToPoint(False, xend, yend, top, midTravel)
    getRightToPoint(False, xstart, ystart, top, midTravel)
    getRightToPoint(False, xstart, ystart, first, slowTravel)
    RightCup(True)
    getRightToPoint(False, xstart, ystart, top, midTravel)
    getRightToPoint(False, xend, yend, top, midTravel)
    getRightToPoint(False, xend, yend, third, slowTravel)
    RightCup(False)
    
    #fire appears at the end when succefully completed
    fireArr = np.array([[1,0,0,xend],
                  [0,1,0,yend],
                  [0,0,1,table],
                  [0,0,0,1]])
    moveObj(fireArr,clientID,fire)
    getLeftToPoint(False, xend, yend, 1.5, midTravel)
    
    

    
            
            
