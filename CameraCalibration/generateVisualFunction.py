import numpy as np
import math
import random
import time
import threading
import sys


def generateSinFunction(xCrop):


    x=np.linspace(-1,1,xCrop[-1]*2);
    
    X=np.tile(x,(976,1))

    Y=np.tile(x,(xCrop[-1]*2,1))
    Y=Y.T
    Y=Y[-xCrop[2]:-xCrop[2]+976,:]

    if np.shape(Y)[0]<976:
        Y2=np.zeros((np.shape(X)[0]-np.shape(Y)[0],np.shape(X)[1]))
        Y=np.concatenate((Y2,Y),axis = 0)




    fov = math.pi * 220/180.0
    R=np.sqrt(np.power(X,2)+np.power(Y,2));
    circle = R<.99
    R2=R
    R[R>1]=0
    phi2 = (np.arctan2(X,Y))
    theta2 = (fov * R) - math.pi/2


    phi = np.arctan2(np.tan(theta2),np.sin(phi2))
    theta = np.arcsin(np.cos(theta2)*np.cos(phi2))
    dPhi = np.abs((np.roll(phi,1,axis=1)-np.roll(phi,-1,axis=1)))

    dPhi[dPhi>math.pi] = np.abs(dPhi[dPhi>math.pi]-2*math.pi)

    
    #dPhi = dPhi*circle
    dTheta = np.abs((np.roll(theta,-1,axis=0)-np.roll(theta,1,axis=0)))
    dTheta = circle*(dTheta)
    dPhi = dPhi*circle

    circularMask = R2<.98
 
    Vcoscos = np.cos(phi)*np.cos(theta)*dTheta
    Vcossin = np.sin(phi)*np.cos(theta)*dTheta
    Vsin = np.sin(theta)*dTheta
    VcoscosA = np.array(Vcoscos*circularMask)
    VcoscosR = np.array(Vcoscos*dPhi)
    VcossinA = np.array(Vcossin*circularMask)
    VcossinR = np.array(Vcossin*dPhi)
    VsinA = Vsin*dPhi*circularMask
    VsinR = Vsin*dPhi*dTheta
    
    
    #im = np.array(Vcoscos * 255, dtype = np.uint8)
    #cv2.imwrite('/home/pi/imTest/Vcoscos.jpg',im)
    return VcoscosA,VcoscosR,VcossinA,VcossinR,VsinA,VsinR,circularMask,phi,dPhi,theta,dTheta

def sectionCrop(crop):
    xCenter = int(crop[1])
    yCenter = int(crop[2])
    RMax = int(crop[0])
    xMax = xCenter+RMax
    yMax = yCenter+RMax
    xMin = xCenter-RMax
    yMin = yCenter-RMax
    xCrop = [xMin,xMax,yMin,yMax,xCenter,yCenter,RMax]
    return xCrop


def main():
    eyeProp = np.loadtxt('/home/pi/droneSpecs.csv')
    xCrop = sectionCrop(eyeProp)
    VcoscosA,VcoscosR,VcossinA,VcossinR,VsinA,VsinR,circle,phi,dphi,theta,dtheta=generateSinFunction(xCrop)
    np.savetxt('/home/pi/phi.csv',phi,delimiter=",")
    np.savetxt('/home/pi/theta.csv',theta,delimiter=",")
    np.savetxt('/home/pi/dphi.csv',dphi,delimiter=",")
    np.savetxt('/home/pi/dtheta.csv',dtheta,delimiter=",")
    np.savetxt('/home/pi/VcoscosA.csv',VcoscosA,delimiter=",")
    np.savetxt('/home/pi/VcoscosR.csv',VcoscosR,delimiter=",")
    np.savetxt('/home/pi/VcossinA.csv',VcossinA,delimiter=",")
    np.savetxt('/home/pi/VcossinR.csv',VcossinR,delimiter=",")
    np.savetxt('/home/pi/VsinA.csv',VsinA,delimiter=",")
    np.savetxt('/home/pi/VsinR.csv',VsinR,delimiter=",")
    np.savetxt('/home/pi/circle.csv',circle,delimiter=",")





if __name__ == '__main__':
    main()