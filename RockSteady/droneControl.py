import picamera
import picamera.array
from picamera.array import PiRGBAnalysis
#import cv2
import numpy as np
import math
import random
import time
import threading
import sys

from core.bebop import *

def generateSinFunction(xCrop):


    x=np.linspace(-1,1,xCrop[-1]*2);
    
    X=np.tile(x,(976,1))

    Y=np.tile(x,(xCrop[-1]*2,1))
    Y=Y.T
    Y=Y[-xCrop[2]:-xCrop[2]+976,:]

    if np.shape(Y)[0]<976:
        Y2=np.zeros((np.shape(X)[0]-np.shape(Y)[0],np.shape(X)[1]))
        Y=np.concatenate((Y2,Y),axis = 0)




    fov = math.pi/2.0 * 220/180.0
    R=np.sqrt(np.power(X,2)+np.power(Y,2));
    circle = R<.99
    R2=R
    R[R>1]=0
    phi2 = (np.arctan2(X,Y))
    theta2 = (fov * R) - math.pi/2


    phi = np.arctan2(np.sin(theta2),np.sin(phi2)*np.cos(theta2))
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
    return VcoscosA,VcoscosR,VcossinA,VcossinR,VsinA,VsinR,circularMask


class visionAnalyzer(PiRGBAnalysis):
    duV = 0
    dudV = 0
    dpV = 0
    dpdV = 0
    
    i=0
    thresholdRed = 50
    thresholdBlue = 200

    xCrop = []
    t0 = 0
    fov = 220.0/180.0
    circularMask = []
    VcoscosA = []
    VcossinA = []
    VsinA = []
    VcoscosR = []
    VcossinR = []
    VsinR = []
    circle = []


        


    def __init__(self,camera,sectionCrop):
        super(visionAnalyzer,self).__init__(camera)
        self.xCrop = sectionCrop

        print('generateSinFunction')
        self.VcoscosA,self.VcoscosR,self.VcossinA,self.VcossinR,self.VsinA,self.VsinR,self.circle=generateSinFunction(self.xCrop)
        print('generatedSinFunction')


        

    def analyze(self,frame):

        frameC = frame[:,self.xCrop[0]:self.xCrop[1],:]
        t0=time.time()
        
        maskRB = np.subtract(frameC[:,:,2], frameC[:,:,1].astype(np.int16))
        maskRB=maskRB>self.thresholdRed
        maskdRB = (np.roll(maskRB,1,axis=1) != np.roll(maskRB,-1,axis=1))
        n = str(self.i).zfill(5)
        self.duV = np.sum(self.VcoscosR[maskRB])
        self.dudV = np.sum(self.VcoscosA[maskdRB])/2.0
        self.dpV = np.sum(self.VcossinR[maskRB])
        self.dpdV = np.sum(self.VcossinA[maskdRB])/2.0
        print('frame : '+n)
        print('vision from frame : '+str((self.duV,self.dpV,self.dudV,self.dpdV)))

        self.i=self.i+1
        t1=time.time()
        print('fps : ' + str(int(1/(t1-self.t0))))
        self.t0 = t1

        #im = np.array(Vcoscos * 255, dtype = np.uint8)
        #cv2.imwrite('/home/pi/imTest/Vcoscos.jpg',im)




class vision:
    Vu = 0
    Vp = 0
    dVu = 0
    dVp = 0

    xCrop=[]


    def sectionCrop(self,crop):
        xCenter = int(crop[1])
        yCenter = int(crop[2])
        RMax = int(crop[0])
        xMax = xCenter+RMax
        yMax = yCenter+RMax
        xMin = xCenter-RMax
        yMin = yCenter-RMax
        self.xCrop = [xMin,xMax,yMin,yMax,xCenter,yCenter,RMax]

    def visionUpdater(self):

        with picamera.PiCamera() as camera:

            camera.resolution = (1296,976)
            #camera.zoom = ( .3563 , 0.2875 , 228/640 , 228/480 )
            camera.framerate = 10
            camera.iso = 800
            camera.shutter_speed = 50000
            camera.awb_mode = 'off'
            camera.awb_gains=(2.5,6)
            #camera.exposure_speed = 100
            #camera.exposure_mode = 'night'
            camera.exposure_compensation = 20
        
            firstRound = True
            running =True 
    
            try:
                k = 0

                with visionAnalyzer(camera,self.xCrop) as anal:
                    camera.start_recording(anal, 'bgr')
                #for frame in enumerate(camera.capture_continuous(rawCapture, 'rgb')):#,resize=(228,228))):
                    while running:
                        
                        k=k+1
                        #print('analog gain : '+str(camera.analog_gain)+' digital_gain : '+str(camera.digital_gain))
                        camera.wait_recording(1.0/camera.framerate)
                        if camera.analog_gain >7 and camera.digital_gain > 1 and firstRound:
                            
                            camera.exposure_mode = 'off'
                            camera.awb_mode = 'off'
                            b=0
                            r=0

                            #camera.awb_gains=(8, 4)
                            firstRound = False
 
                        else:
                            self.Vu =anal.duV
                            self.Vp =anal.dpV
                            self.dVu=anal.dudV
                            self.dVp=anal.dpdV 
                        if not firstRound and False:
                            r=r+.1
                            if r>8:
                                r=0
                                b=b+.1
                            if b>8:
                                break
                            camera.awb_gains=(r,b)
                            print(camera.awb_gains)
            except:
                pass

    def start(self):
        print('start to see the world')
        visionThread = threading.Thread(target = self.visionUpdater, args=())
        visionThread.daemon = True
        visionThread.start()  
    
    def __init__(self):  
        print('vision is intialized -- Ready to start')



class droneController:
    drone = []
    zFlight = 0.5
    Vu = 0
    Vp = 0
    dVu = 0
    dVp = 0
    droneConnected =False

    au = 0
    ru = 0
    ap = 0.1
    rp = -1
    um = 0
    pm = 1
    v0 = 0
    a = .1 

#    if len(sys.argv) > 2:
#        metalog = MetaLog( filename=sys.argv[2] )
#    if len(sys.argv) > 3 and sys.argv[3] == 'F':
#        disableAsserts()

    def updateParameters(self,au=.1,ru=-1,ap=.1,rp=-1,um=1,pm=1,v0=1,a=.1):
        self.au = au
        self.ru = ru
        self.ap = ap
        self.rp = rp
        self.um = um
        self.pm = pm
        self.v0 = v0
        self.a = a


    def TakeOff(self):
        if self.droneConnected:
            self.drone.takeoff()
            #self.drone.flyToAltitude(zFlight)

    def emergencyLanding(self):
        if self.droneConnected:
            self.drone.emergency()

    def landing(self):
        if self.droneConnected:
            self.drone.land()


    def move(self):
        roll = 0
        pitch = self.um*(self.ru*self.Vu+self.au*self.dVu)
        yaw = self.pm*(self.rp*self.Vp+self.ap*self.dVp)
        gaz = 0

        print('pitch : '+str(pitch))
        print('yaw  : '+str(yaw))
        if self.droneConnected:
            self.drone.update(cmd=movePCMDCmd(active=True, roll=roll, pitch=pitch, yaw=yaw, gaz=gaz))


    def updateVision(self,Vu,Vp,dVu,dVp):
        self.Vu = Vu
        self.Vp = Vp
        self.dVu = dVu
        self.dVp = dVp
        #print('Vision : '+str((self.Vu,self.Vp,self.dVu,self.dVp)))


    def start(self):
        print('start the drone connection and control')
        try:

            metalog=None
            self.drone = Bebop( metalog=metalog )
            self.drone.trim()
            self.droneConnected = True

        except:
            self.droneConnected = False
            


    def __init__(self):
        self.zFlight = 0.5
        metalog=None
        print('Drone Control is intialized -- Ready to start')


