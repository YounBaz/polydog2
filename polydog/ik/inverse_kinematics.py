#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from mpl_toolkits import mplot3d
import numpy as np
from math import *
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray
import time
rospy.init_node('joint_state_publisher')
pub = rospy.Publisher('joint_states', JointState, queue_size=10)
def setupView(limit):
    ax = plt.axes(projection="3d")
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    ax.set_xlabel("X")
    ax.set_ylabel("Z")
    ax.set_zlabel("Y")
    return ax

setupView(600).view_init(elev=12., azim=28)

omega =  pi/4
phi =0
psi = 0

xm=0
ym=0
zm=0

l1=30
l2=0
l3=129.5
l4=125

L = 286
W = 205

Lp=np.array([[143,-200,135,1],[143,-200,-70,1],[-143,-200,135,1],[-143,-200,-70,1]])

sHp=np.sin(pi/2)
cHp=np.cos(pi/2)

Lo=np.array([0,0,0,1])
while not rospy.is_shutdown():
    # Calculate joint angles
    # ...
    def bodyIK(omega,phi,psi,xm,ym,zm):
        Rx = np.array([[1,0,0,0],
                       [0,np.cos(omega),-np.sin(omega),0],
                       [0,np.sin(omega),np.cos(omega),0],[0,0,0,1]])
        Ry = np.array([[np.cos(phi),0,np.sin(phi),0],
                       [0,1,0,0],
                       [-np.sin(phi),0,np.cos(phi),0],[0,0,0,1]])
        Rz = np.array([[np.cos(psi),-np.sin(psi),0,0],
                       [np.sin(psi),np.cos(psi),0,0],[0,0,1,0],[0,0,0,1]])
        Rxyz=Rx@Ry@Rz

        T = np.array([[0,0,0,xm],[0,0,0,ym],[0,0,0,zm],[0,0,0,0]])
        Tm = T+Rxyz

        return([Tm @ np.array([[cHp,0,sHp,L/2],[0,1,0,0],[-sHp,0,cHp,W/2],[0,0,0,1]]),
               Tm @ np.array([[cHp,0,sHp,L/2],[0,1,0,0],[-sHp,0,cHp,-W/2],[0,0,0,1]]),
               Tm @ np.array([[cHp,0,sHp,-L/2],[0,1,0,0],[-sHp,0,cHp,W/2],[0,0,0,1]]),
               Tm @ np.array([[cHp,0,sHp,-L/2],[0,1,0,0],[-sHp,0,cHp,-W/2],[0,0,0,1]])
               ])

    def legIK(point):
        (x,y,z)=(point[0],point[1],point[2])
        F=sqrt(x**2+y**2-l1**2)
        G=F-l2  
        H=sqrt(G**2+z**2)
        theta1=-atan2(y,x)-atan2(F,-l1)

        D=(H**2-l3**2-l4**2)/(2*l3*l4)
        theta3=acos(D) 

        theta2=atan2(z,G)-atan2(l4*sin(theta3),l3+l4*cos(theta3))
        print(theta1,theta2,theta3)
        return(theta1,theta2,theta3)

    def calcLegPoints(angles):
        (theta1,theta2,theta3)=angles
        theta23=theta2+theta3

        T0=Lo
        T1=T0+np.array([-l1*cos(theta1),l1*sin(theta1),0,0])
        T2=T1+np.array([-l2*sin(theta1),-l2*cos(theta1),0,0])
        T3=T2+np.array([-l3*sin(theta1)*cos(theta2),-l3*cos(theta1)*cos(theta2),l3*sin(theta2),0])
        T4=T3+np.array([-l4*sin(theta1)*cos(theta23),-l4*cos(theta1)*cos(theta23),l4*sin(theta23),0])

        return np.array([T0,T1,T2,T3,T4])

    def drawLegPoints(p):
        plt.plot([x[0] for x in p],[x[2] for x in p],[x[1] for x in p], 'k-', lw=3)
        plt.plot([p[0][0]],[p[0][2]],[p[0][1]],'bo',lw=2)
        plt.plot([p[4][0]],[p[4][2]],[p[4][1]],'ro',lw=2)    

    def drawLegPair(Tl,Tr,Ll,Lr):
        Ix=np.array([[-1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        drawLegPoints([Tl@x for x in calcLegPoints(legIK(np.linalg.inv(Tl)@Ll))])
        drawLegPoints([Tr@Ix@x for x in calcLegPoints(legIK(Ix@np.linalg.inv(Tr)@Lr))])

    def drawRobot(Lp,angles,center):
        (omega,phi,psi)=angles
        (xm,ym,zm)=center

        FP=[0,0,0,1]
        (Tlf,Trf,Tlb,Trb)= bodyIK(omega,phi,psi,xm,ym,zm)
        CP=[x@FP for x in [Tlf,Trf,Tlb,Trb]]

        CPs=[CP[x] for x in [0,1,3,2,0]]
        plt.plot([x[0] for x in CPs],[x[2] for x in CPs],[x[1] for x in CPs], 'bo-', lw=2)

        drawLegPair(Tlf,Trf,Lp[0],Lp[1])
        drawLegPair(Tlb,Trb,Lp[2],Lp[3])
        (theta11,theta12,theta13)=legIK(np.linalg.inv(Tlf)@Lp[0])
        (theta21,theta22,theta23)=legIK(np.linalg.inv(Trf)@Lp[1])
        (theta31,theta32,theta33)=legIK(np.linalg.inv(Tlb)@Lp[2])
        (theta41,theta42,theta43)=legIK(np.linalg.inv(Trb)@Lp[3])
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['shoulder1', 'hip1', 'knee1', 'shoulder2', 'hip2', 'knee2', 'shoulder3', 'hip3', 'knee3', 'shoulder4', 'hip4', 'knee4']  # Replace with your actual joint names
        joint_state.position = [0.012-theta31, theta32-1.57, -3.925+theta33, 0.012-theta11, theta12-1.57, -3.925+theta13, theta21, -theta22-1.57, -3.925+theta23, theta41, -theta42-1.57, -3.925+theta43]  # Replace with your actual angle

        # Publish the joint states
        pub.publish(joint_state)
    
    
    #def sit():
        #for i in range (0,-90,1):
    drawRobot(Lp,(0,0,0),(0,0,0))
    #plt.show()
    # Create a JointState message


    # Sleep for a short duration to control the publishing rate
    rospy.sleep(0.01)  # Adjust as needed
