import picamera
import picamera.array
from picamera.array import PiRGBAnalysis
import cv2
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


    
    

    R=np.power(X,2)+np.power(Y,2)
    circle = R<.99
    R[R>1]=0
    phi = 1.2*(np.arctan2(X,1-R))-math.pi/2
    theta = np.arctan2(Y,np.sqrt(1-np.power(Y,2)))
    dPhi = (np.roll(phi,1,axis=1)-np.roll(phi,-1,axis=1))/2.0
    #dPhi = dPhi*circle
    dTheta = (np.roll(theta,-1,axis=0)-np.roll(theta,1,axis=0))/2.0
    dTheta = circle*(dTheta*(dTheta>0))
    dPhi = dPhi*circle
    circularMask = R<.98

    Vcoscos = np.cos(phi)*np.cos(theta)*circle
    Vcossin = np.sin(phi)*np.cos(theta)*circle
    Vsin = np.sin(theta)*circle
    VcoscosA = np.array(Vcoscos*dTheta*circularMask)
    VcoscosR = np.array(Vcoscos*dTheta*dPhi)
    VcossinA = np.array(Vcossin*dTheta*circularMask)
    VcossinR = np.array(Vcossin*dTheta*dPhi)
    VsinA = Vsin*np.power(dPhi,2)*dTheta*circularMask
    VsinR = Vsin*np.power(dPhi,1)*dTheta*dPhi
    

    return VcoscosA,VcoscosR,VcossinA,VcossinR,VsinA,VsinR,circularMask


class visionAnalyzer(PiRGBAnalysis):
    duV = 0
    dudV = 0
    dpV = 0
    dpdV = 0
    
    i=0
    thresholdRed = 50
    thresholdBlue = 255

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
        #self.circularMask = np.zeros((976,self.xCrop[-1]*2),np.uint8)
        print('generateSinFunction')
        self.VcoscosA,self.VcoscosR,self.VcossinA,self.VcossinR,self.VsinA,self.VsinR,self.circle=generateSinFunction(self.xCrop)
        print('generatedSinFunction')
        #cv2.circle(self.circularMask,((self.xCrop[-1]),self.xCrop[5]),(self.xCrop[-1]),1,thickness=-1)

        

    def analyze(self,frame):


        #self.threshold=self.threshold+1
        #if self.threshold>200:
        #    self.threshold = 0
        frameC = frame[:,self.xCrop[0]:self.xCrop[1],:]
        if False:
            frameC = cv2.bitwise_and(frameC,frameC,mask = self.circularMask)
            t0=time.time()
            ret,thres = cv2.threshold(frameC[:,:,2],self.thresholdRed,255,cv2.THRESH_BINARY)
            ret,thres2 = cv2.threshold(frameC[:,:,0],self.thresholdBlue,255,cv2.THRESH_BINARY_INV)
            thres = cv2.multiply(thres,thres2)
            t1=time.time()
            print('opencv thresholding : '+str(t1-t0))
            t0=time.time()
        redMask = frameC[:,:,2]>self.thresholdRed
        blueMask = frameC[:,:,0]<self.thresholdBlue

        maskRB = np.array(redMask*blueMask)
        maskdRB = (np.roll(maskRB,1,axis=1) != np.roll(maskRB,-1,axis=1))
        #t1=time.time()
        #print('numpy thresholding  : '+str(t1-t0))
        #kernel = np.ones((5,5),np.uint8)
        #thres = cv2.erode(thres,kernel,iterations = 1)
        #thres = cv2.dilate(thres,kernel,iterations = 3)
        #t0=time.time()
        if False:
            im2,contours,hierarchy = cv2.findContours(thres ,cv2.RETR_LIST ,cv2.CHAIN_APPROX_SIMPLE)
            #cv2.drawContours(thres,contours,-1,(0,0,255),2)


            for contour in contours:
                if len(contour)>20:
                    contX = ((contour[:,0,0]-self.xCrop[-1])/float(self.xCrop[-1]))
                    contY = ((contour[:,0,1]-self.xCrop[5])/float(self.xCrop[-1]))
                    phi = (self.fov*np.arctan2(contX,np.sqrt(1-(np.power(contX,2)+np.power(contY,2)))))-math.pi/2
                    theta = np.arctan2(contY,np.sqrt(1-(np.power(contY,2))))
                    phiMax = np.max(phi)
                    phiMin = np.min(phi)
                    self.duV = self.duV + (math.sin(phiMax)-math.sin(phiMin))
                    self.dpV = self.dpV - (math.cos(phiMax)-math.cos(phiMin))
                    if phiMin>-.95*math.pi:
                        self.dudV = self.dudV + (math.cos(phiMin))
                        self.dpdV = self.dpdV + (math.sin(phiMin))
                    if phiMax<-.05*math.pi:
                        self.dudV = self.dudV + (math.cos(phiMax))
                        self.dpdV = self.dpdV + (math.sin(phiMax))


            t1=time.time()
            print('opencv integration : '+str(t1-t0))
        
        #t0=time.time()

        self.duV = np.sum(self.VcoscosA[maskRB])
        self.dudV = np.sum(self.VcoscosR[maskdRB])
        self.dpV = np.sum(self.VcossinA[maskRB])
        self.dpdV = np.sum(self.VcossinR[maskdRB])
        #print('vision from frame : '+str((self.duV,self.dpV,self.dudV,self.dpdV)))
        #t1=time.time()
        #print('numpy integration  : '+str(t1-t0))
        #t0=time.time()
        A=self.VcoscosA
        cv2.imwrite('/home/pi/imTest/front'+str(self.i)+'.jpg',A)
        cv2.imwrite('/home/pi/imTest/side'+str(self.i)+'.jpg',maskRB)
        frameC[maskRB]=0
        cv2.imwrite('/home/pi/imTest/image'+str(self.i)+'.jpg',frameC)
        #print('image'+str(self.i)+'.jpg')
        self.i=self.i+1
        t1=time.time()
        print('fps : ' + str(int(1/(t1-self.t0))))
        self.t0 = t1
        #print('thres : ' + str(self.threshold))



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
            camera.shutter_speed = 100000
            camera.awb_mode = 'off'
            camera.awb_gains=(8,2)
            #camera.exposure_speed = 100
            #camera.exposure_mode = 'night'
            camera.exposure_compensation = 25
        
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
        #print('pitch : '+str(pitch))
        #print('yaw  : '+str(yaw))
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


