from variables import *

def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])
def dskew(x):
    return np.array[[x[2][1]],[x[0][2]],[x[1][0]]]
def vskew(x):
    return np.array([[0,-x[2],x[1],x[3]],
                     [x[2],0,-x[0],x[4]],
                     [-x[1],x[0],0,x[5]],
                     [0,0,0,0]])  
def dvskew(x):
    return np.array([[x[2][1]],
                    [x[0][2]],
                    [x[1][0]],
                    [x[0][3]],
                    [x[1][3]],
                    [x[2][3]]])
    
  
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


def exp(s,t):
    return sl.expm(vskew(s)*t)
    
    
def admaker(x):
    temp = np.zeros((6,6))
    temp[:3,:3] = np.copy(x[:3,:3])
    temp[3:,:3] = np.dot(skew(x[:3,3]), x[:3,:3])
    temp[3:,3:] = np.copy(x[:3,:3])
    return temp


def rad(x):
    return math.radians(x)

def sind(x):
    return np.sin(rad(x))

def cosd(x):
    return np.cos(rad(x))

def TtoM(theta, s,M):
    T = np.identity(4)
    for i in range(theta.size):
        T = T.dot(sl.expm(vskew(s[:,i])*theta[i,0]))
    return T.dot(M)

def td(J, V):
    return nl.inv(np.transpose(J)@J+.1*np.identity(J.shape[1]))@np.transpose(J)@V


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
    
    vrep.simxSetObjectPosition(clientID, objHandle, -1, p, vrep.simx_opmode_streaming)
    vrep.simxSetObjectOrientation(clientID, objHandle, -1, E, vrep.simx_opmode_oneshot)

def leftArmPose(thetal):
    #left arm (facing towards baxter)
    thetaLeft = np.array([[math.radians(thetal[0])], [math.radians(thetal[1])], [math.radians(thetal[2])], [math.radians(thetal[3])], [math.radians(thetal[4])], [math.radians(thetal[5])], [math.radians(thetal[6])], [math.radians(thetal[7])]])
    ML = np.array([[1,0,0,.0815],
                  [0,1,0,1.2993],
                  [0,0,1,1.2454],
                  [0,0,0,1]])
        
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
    
    
    finalLeft = TtoM(thetaLeft,s,ML )
    return finalLeft


def rightArmPose(thetar):
    #right arm
    thetaRight= np.array([[math.radians(thetar[0])], [math.radians(thetar[1])], [math.radians(thetar[2])], [math.radians(thetar[3])], [math.radians(thetar[4])], [math.radians(thetar[5])], [math.radians(thetar[6])], [math.radians(thetar[7])]])
    MR = np.array([[1,0,0,.0818],
                  [0,1,0,-1.2929],
                  [0,0,1,1.2454],
                  [0,0,0,1]])
        
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
    
    
    finalRight = TtoM(thetaRight,s,MR )
    return finalRight

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



def check(theta, limits):
    count = 0
    for ele in theta:
        if(count == 0):
            count = count + 1
            continue
        if(ele > limits[count][0] and ele < limits[count][1]):
            #print(str(ele) + " " + str(limits[count][0])+ " " + str(limits[count][1])+ " true")
            count = count + 1
            continue
        else:
            #print(str(ele) + " " + str(limits[count][0])+ " " + str(limits[count][1])+ " false")
            return False
    return True



def invLeftArm(xl, yl, zl):
    #start at zero position
    
    r=0
    rpose = rightArmPose(0,r,r,r,r,r,r,r)   
    l=0
    lpose = leftArmPose(0,l,l,l,l,l,l,l)
    moveObj(rpose, clientID, objHandleRightTheoretical)
    moveObj(lpose, clientID, objHandleLeftTheoretical)
    for i in range(0,7):
        vrep.simxSetJointTargetPosition(clientID, rightArm[i], math.radians(r), vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, leftArm[i], math.radians(l), vrep.simx_opmode_oneshot)
    

    leftTip = int(vrep.simxGetObjectHandle(clientID, 'Baxter_leftArm_tip', vrep.simx_opmode_blocking)[1])
    T_2in0 = np.array([[1, 0, 0, xl], [0, 1, 0, yl], [0, 0, 1, zl], [0.00000000, 0.00000000, 0.00000000, 1.00000000]])
    moveObj(T_2in0, clientID, objHandleLeftTheoretical)
    LeftLimits = [(math.radians(-168), math.radians(-168+336)), (math.radians(-96.5), math.radians(-96.5+191)), (math.radians(-121), math.radians(-121+179)), (math.radians(-173), math.radians(-173+346)), (math.radians(0), math.radians(0+148)), (math.radians(-173.3), math.radians(-173.3+346.5)), (math.radians(-88), math.radians(-88+206)),(math.radians(-173.3), math.radians(-173.3+346.5))]
        
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
            thetadot = td(J,V0, thetaL)
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
                if(failCount >= 7):
                    print("Position Unreachable")
                    print()
                    return
        # print(repr(thetaL))
        #checking if the angles are within limit of their respective joints
        if(check(thetaL, LeftLimits)):
            time.sleep(1) 
            vrep.simxSetJointTargetPosition(clientID, rotJoint, thetaL[0], vrep.simx_opmode_oneshot)
            for i in range(0,7):
                vrep.simxSetJointTargetPosition(clientID, leftArm[i], thetaL[i+1], vrep.simx_opmode_oneshot)
                time.sleep(.5)
            #leftPose = leftArmPose(thetaL[0], thetaL[1], thetaL[2],thetaL[3],thetaL[4],thetaL[5],thetaL[6],thetaL[7],)
            time.sleep(.5)
            leftEndTip = vrep.simxGetObjectPosition(clientID, leftTip, -1, vrep.simx_opmode_blocking)[1]
            if(nl.norm(T_2in0[:3,3]-np.array(leftEndTip))> 0.01):
                #checking final pose, in case any obsticles got in the way (even though joint angles were okay.)
                print("Hmmmm, the end of the baxter arm is not where it should be. Collision detected")
                print("Difference: " + str(nl.norm(T_2in0[:3,3]-np.array(leftEndTip))))
                print()
                thetaL = np.array([[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)]])
                thetadot = np.array([100])
                TL_1in0 = TtoM(thetaL,SL,ML)
                J = Jmaker(thetaL,SL)
                V0 = dvskew(sl.logm(T_2in0.dot(nl.inv(TL_1in0))))
                V = dvskew(sl.logm(T_2in0.dot(nl.inv(TL_1in0))))
                continue
            print("BINGO! Position Achieved!")
            print()
            time.sleep(3)
            break
        else:
            print("Came up with set of thetas where some of them where out of range, trying again with different starting position")
            print()
            #try different starting orientation to get different end angles that will hopefully meet the angle limits of the joints
            thetaL = np.array([[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)]])
            thetadot = np.array([100])
            TL_1in0 = TtoM(thetaL,SL,ML)
            J = Jmaker(thetaL,SL)
            V0 = dvskew(sl.logm(T_2in0.dot(nl.inv(TL_1in0))))
            V = dvskew(sl.logm(T_2in0.dot(nl.inv(TL_1in0))))
        
        
        
def invRightArm(xr, yr, zr):
    #start at zero position
    
    r=0
    rpose = rightArmPose(0,r,r,r,r,r,r,r)   
    l=0
    lpose = leftArmPose(0,l,l,l,l,l,l,l)
    moveObj(rpose, clientID, objHandleRightTheoretical)
    moveObj(lpose, clientID, objHandleLeftTheoretical)
    for i in range(0,7):
        vrep.simxSetJointTargetPosition(clientID, rightArm[i], math.radians(r), vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, leftArm[i], math.radians(l), vrep.simx_opmode_oneshot)
    

    rightTip = int(vrep.simxGetObjectHandle(clientID, 'Baxter_rightArm_tip', vrep.simx_opmode_blocking)[1])
    T_2in0 = np.array([[1, 0, 0, xr], [0, 1, 0, yr], [0, 0, 1, zr], [0.00000000, 0.00000000, 0.00000000, 1.00000000]])
    moveObj(T_2in0, clientID, objHandleRightTheoretical)
    rightLimits = [(math.radians(-168), math.radians(-168+336)), (math.radians(-96.5), math.radians(-96.5+191)), (math.radians(-121), math.radians(-121+179)), (math.radians(-173), math.radians(-173+346)), (math.radians(0), math.radians(0+148)), (math.radians(-173.3), math.radians(-173.3+346.5)), (math.radians(-88), math.radians(-88+206)),(math.radians(-173.3), math.radians(-173.3+346.5))]
        
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
            thetadot = td(J,V0, thetaR)
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
                if(failCount >= 7):
                    print("Position Unreachable")
                    print()
                    return
        # print(repr(thetaR))
        #checking if the angles are within limit of their respective joints
        if(check(thetaR, rightLimits)):
            time.sleep(1) 
            vrep.simxSetJointTargetPosition(clientID, rotJoint, thetaR[0], vrep.simx_opmode_oneshot)
            for i in range(0,7):
                vrep.simxSetJointTargetPosition(clientID, rightArm[i], thetaR[i+1], vrep.simx_opmode_oneshot)
                time.sleep(.5)
            time.sleep(.5)
            leftEndTip = vrep.simxGetObjectPosition(clientID, rightTip, -1, vrep.simx_opmode_blocking)[1]
            if(nl.norm(T_2in0[:3,3]-np.array(leftEndTip))> 0.01):
                #checking final pose, in case any obsticles got in the way (even though joint angles were okay.)
                print("Hmmmm, the end of the baxter arm is not where it should be. Collision detected")
                print("Difference: " + str(nl.norm(T_2in0[:3,3]-np.array(leftEndTip))))
                print()
                thetaR = np.array([[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)]])
                thetadot = np.array([100])
                TR_1in0 = TtoM(thetaR,SR,MR)
                J = Jmaker(thetaR,SR)
                V0 = dvskew(sl.logm(T_2in0.dot(nl.inv(TR_1in0))))
                V = dvskew(sl.logm(T_2in0.dot(nl.inv(TR_1in0))))
                continue
            print("BINGO! Position Achieved!")
            print()
            time.sleep(3)
            break
        else:
            
            print("Came up with set of thetas where some of them where out of range, trying again with different starting position")
            print()
            #try different starting orientation to get different end angles 
            thetaR = np.array([[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)],[random.uniform(-3.14, 3.14)]])
            thetadot = np.array([100])
            TR_1in0 = TtoM(thetaR,SR,MR)
            J = Jmaker(thetaR,SR)
            V0 = dvskew(sl.logm(T_2in0.dot(nl.inv(TR_1in0))))
            V = dvskew(sl.logm(T_2in0.dot(nl.inv(TR_1in0)))) 

    
def moveLeft(thetas):
    vrep.simxSetJointTargetPosition(clientID, rotJoint, math.radians(thetas[0]), vrep.simx_opmode_oneshot) 
    for i in range(0,7):
        vrep.simxSetJointTargetPosition(clientID, leftArm[i], math.radians(thetas[i+1]), vrep.simx_opmode_oneshot) 
        
def moveRight(thetas):
    vrep.simxSetJointTargetPosition(clientID, rotJoint, math.radians(thetas[0]), vrep.simx_opmode_oneshot) 
    for i in range(0,7):
        vrep.simxSetJointTargetPosition(clientID, rightArm[i], math.radians(thetas[i+1]), vrep.simx_opmode_oneshot) 


def endSim():
    input("Press Enter to end the simulation.")
    # Stop simulation
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
    
    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)
    
    # Close the connection to V-REP
    vrep.simxFinish(clientID)

    
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
                    
def detectCollisionLeft(thetal):
    
    LeftLimits = [(math.radians(-168), math.radians(-168+336)), (math.radians(-96.5), math.radians(-96.5+191)), (math.radians(-121), math.radians(-121+179)), (math.radians(-173), math.radians(-173+346)), (math.radians(0), math.radians(0+148)), (math.radians(-173.3), math.radians(-173.3+346.5)), (math.radians(-88), math.radians(-88+206)),(math.radians(-173.3), math.radians(-173.3+346.5))]
    if(check(thetal,LeftLimits)):
        print("Left: Joint angles out of range")
        return True
    
    theta = np.zeros(thetal.size)
    for i in range(thetal.size):
        theta[i] = math.radians(thetal[i])
    col = False
    
    S = leftArmS()
    M = ML
    r_robot = np.array([[0,.2,.05,.05,.05,.05,.05,.05,.05,.05]])
    p_obstacle = np.array([[.95], [-.225], [1]])
    r_obstacle = np.array([[.5]])
    
    coords = np.zeros((3,8))   
    coords[:,0] = np.array([[.0177],[.0032],[.8777]]).reshape((3))
    coords[:,1] = np.array([[.0815],[.2622],[1.0540]]).reshape((3))
    coords[:,2] = np.array([[.0815],[.3312],[1.3244]]).reshape((3))
    coords[:,3] = np.array([[.0815],[.4332],[1.3244]]).reshape((3))
    coords[:,4] = np.array([[.0815],[.6956],[1.2554]]).reshape((3))
    coords[:,5] = np.array([[.0815],[.7992],[1.2554]]).reshape((3))
    coords[:,6] = np.array([[.0815],[1.0699],[1.2454]]).reshape((3))
    coords[:,7] = np.array([[.0815],[1.1859],[1.2454]]).reshape((3))
        
    #print(coords)
    
    
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
            #dist = np.sqrt((centers[0][l] - centers[0][y])**2 + (centers[1][l] - centers[1][y])**2 + (centers[2][l] - centers[2][y])**2)
            if dist < r[0][l]+r[0][y]:
                if y == r[0].size-1 or l == r[0].size-1:
                    print("Left: collision with outside object")
                    col = True
                    return col
                else:
                    print("Left: collision with self")
                    col = True
                    return col
    if(not col):
        print("Left: no collisions detected")

    return col

def detectCollisionRight(thetal):
    
    rightLimits = [(math.radians(-168), math.radians(-168+336)), (math.radians(-96.5), math.radians(-96.5+191)), (math.radians(-121), math.radians(-121+179)), (math.radians(-173), math.radians(-173+346)), (math.radians(0), math.radians(0+148)), (math.radians(-173.3), math.radians(-173.3+346.5)), (math.radians(-88), math.radians(-88+206)),(math.radians(-173.3), math.radians(-173.3+346.5))]
    if(check(thetal,rightLimits)):
        print("Right: Joint angles out of range")
        return True
    
    theta = np.zeros(thetal.size)
    for i in range(thetal.size):
        theta[i] = math.radians(thetal[i])
    col = False

    S = rightArmS()
    M = MR
    r_robot = np.array([[0,.2,.05,.05,.05,.05,.05,.05,.05,.05]])
    p_obstacle = np.array([[.95], [-.225], [1]])
    r_obstacle = np.array([[.5]])
    
    coords = np.zeros((3,8))   
    coords[:,0] = np.array([[.0177],[.0032],[.8777]]).reshape((3))
    coords[:,1] = np.array([[.0818],[-.2558],[1.0540]]).reshape((3))
    coords[:,2] = np.array([[.0818],[-.3248],[1.3244]]).reshape((3))
    coords[:,3] = np.array([[.0818],[-.4268],[1.3244]]).reshape((3))
    coords[:,4] = np.array([[.0818],[-.6892],[1.2554]]).reshape((3))
    coords[:,5] = np.array([[.0818],[-.7928],[1.2554]]).reshape((3))
    coords[:,6] = np.array([[.0818],[-1.0635],[1.2454]]).reshape((3))
    coords[:,7] = np.array([[.0818],[-1.1795],[1.2454]]).reshape((3))
        
    #print(coords)
    
    
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
            #dist = np.sqrt((centers[0][l] - centers[0][y])**2 + (centers[1][l] - centers[1][y])**2 + (centers[2][l] - centers[2][y])**2)
            if dist < r[0][l]+r[0][y]:
                if y == r[0].size-1 or l == r[0].size-1:
                    print("Right: collision with outside object")
                    col = True
                    return col
                else:
                    print("Right: collision with self")
                    col = True
                    return col
    if(not col):
        print("Right: no collisions detected")

    return col

def totalLeft(larray):
    lpose = leftArmPose(larray)
    moveLeft(larray)
    placeLeftJoints(larray)
    moveObj(lpose, clientID, objHandleLeftTheoretical)
    detectCollisionLeft(larray)
    
def totalRight(rarray):
    rpose = rightArmPose(rarray)
    moveRight(rarray)
    placeRightJoints(rarray)
    moveObj(rpose, clientID, objHandleRightTheoretical)
    detectCollisionRight(rarray)