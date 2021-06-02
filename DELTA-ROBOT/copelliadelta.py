# -*- coding: utf-8 -*-

import numpy as np
import scipy.linalg as linalg
import matplotlib.pyplot as plt
import sim
import time
    
#CONEXION DE SPYDER CON COPPELIA
sim.simxFinish(-1) # Just in case, close all opened connections

clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5)

#DEGREES OF FREEDOM
dof=3
#INITIAL AND FINAL VALUES FOR POSITION, VELOCITY ,ACCELERATION AND TIME
q0=np.array([.0,.0,.0])
v0=np.array([.0,.0,.0])
ac0=np.array([.0,.0,.0])
t0=0

q1=np.array([.5,.6,.65])
v1=np.array([.0,.0,.0])
ac1=np.array([.0,.0,.0])
tf=1

t=np.linspace(t0,tf,100*(tf-t0))
s=len(t)
c=np.ones(s)
#LISTS FOR COEFFICIENTS
qd_l=[]
vd_l=[]
ad_l=[]

#MOVIMIENTO DEL ROBOT
#DECLARACION DE CADA ARTICULACION COMO OBJETO MANIPULABLE
res,joint1=sim.simxGetObjectHandle(clientID,'joint1',sim.simx_opmode_blocking)
res,joint2=sim.simxGetObjectHandle(clientID,'joint2',sim.simx_opmode_blocking)
res,joint3=sim.simxGetObjectHandle(clientID,'joint3',sim.simx_opmode_blocking)

#DECLARACION DE CADA JUNTA
DELTA1=[joint1]
DELTA2=[joint2]
DELTA3=[joint3]

#MATRIX TO FORM THE POLYNOMIAL EQUATION
M=np.array([[1, t0, t0*t0, t0*t0*t0, t0*t0*t0*t0, t0*t0*t0*t0*t0],
   [0, 1, 2*t0, 3*t0*t0, 4*t0*t0*t0, 5*t0*t0*t0*t0],
   [0, 0, 2, 6*t0, 12*t0*t0, 20*t0*t0*t0],     
   [1, tf, tf*tf, tf*tf*tf, tf*tf*tf*tf, tf*tf*tf*tf*tf],
   [0, 1, 2*tf, 3*tf*tf, 4*tf*tf*tf, 5*tf*tf*tf*tf],
   [0, 0, 2, 6*tf, 12*tf*tf, 20*tf*tf*tf]])
    
#FOR CYCLE TO COMPUTE COEFFICIENTS
for i in range(dof):
    
    #VALUES KNOWNS
    B=np.array([[q0[i]],
                [v0[i]],
                [ac0[i]],
                [q1[i]],
                [v1[i]],
                [ac1[i]]])
    
    #ARRAY OF COEFFICIENTS
    A=linalg.inv(M)@B
    #TRAJECTORIES FOR POSITION, VELOCITY AND ACCELERTION
    qd= A[0,0]*c + A[1,0]*t + A[2,0]*t*t + A[3,0]*t*t*t + A[4,0]*t*t*t*t + A[5,0]*t*t*t*t*t
    vd= A[1,0]*c+ 2*A[2,0]*t + 3*A[3,0]*t*t + 4*A[4,0]*t*t*t + 5*A[5,0]*t*t*t*t
    ad= 2*A[2,0]*c + 6*A[3,0]*t + 12*A[4,0]*t*t + 20*A[5,0]*t*t*t
      
    
    #LISTS FOR EACH COEFFICIENT
    qd_l.append(qd)
    vd_l.append(vd)
    ad_l.append(ad)
    
    #PLOTS FOR EACH GRAPH
#POSITION
    plt.figure(1)
    plt.plot(t,qd_l[i])
    plt.title('Position')

#VELOCITY
    plt.figure(2)
    plt.plot(t,vd_l[i])
    plt.title('Velocity')

#ACCELERATION
    plt.figure(3)
    plt.plot(t,ad_l[i])
    plt.title('Acceleration')

qd1=qd_l[0]
qd2=qd_l[1]
qd3=qd_l[2]


#ROBOT IN INITIAL POSITION
for joint in DELTA1:
    sim.simxSetJointTargetPosition(clientID, joint, (q0[0]), sim.simx_opmode_streaming)
    time.sleep(1/100)
for joint in DELTA2:
    sim.simxSetJointTargetPosition(clientID, joint, (q0[1]), sim.simx_opmode_streaming)
    time.sleep(1/100)
for joint in DELTA3:
    sim.simxSetJointTargetPosition(clientID, joint, (q0[2]), sim.simx_opmode_streaming)
    time.sleep(1/100)

time.sleep(1)

#MOVE TO EACH POSITION
for t2 in range(s):
    for joint in DELTA1:
        sim.simxSetJointTargetPosition(clientID, joint, (qd1[t2]), sim.simx_opmode_streaming)
        time.sleep(1/100)
    for joint in DELTA2:
        sim.simxSetJointTargetPosition(clientID, joint, (qd2[t2]), sim.simx_opmode_streaming)
        time.sleep(1/100)
    for joint in DELTA3:
        sim.simxSetJointTargetPosition(clientID, joint, (qd3[t2]), sim.simx_opmode_streaming)
        time.sleep(1/100)
