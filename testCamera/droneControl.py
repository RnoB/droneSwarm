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
from math import pi

from core.bebop import *


pi2 = pi/2.0

class visionAnalyzer(PiRGBAnalysis):
    duV = 0
    dudV = 0
    dpV = 0
    dpdV = 0
    
    i=0
    threshold = 150

    xCrop = []
    t0 = 0
    circularMask = []
    def __init__(self,camera,sectionCrop):
        super(visionAnalyzer,self).__init__(camera)
        self.xCrop = sectionCrop
        self.circularMask = np.zeros((976,self.xCrop[-1]*2),np.uint8)
        
        cv2.circle(self.circularMask,((self.xCrop[-1]),self.xCrop[5]),(self.xCrop[-1]),1,thickness=-1)

    def analyze(self,frame):
        #cv2.namedWindow('anal',cv2.WINDOW_NORMAL)
        #cv2.resizeWindow('anal',1110,976)
        cv2.namedWindow('frame',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('frame',self.xCrop[-1]*2,976)
        self.duV = 0
        self.dudV = 0
        self.dpV = 0
        self.dpdV = 0
        #self.threshold=self.threshold+1
        #if self.threshold>200:
        #    self.threshold = 0
        frameC = frame[:,self.xCrop[0]:self.xCrop[1],:]
        frameC = cv2.bitwise_and(frameC,frameC,mask = self.circularMask)
        t0 = time.time()
        hsv = cv2.cvtColor(frameC,cv2.COLOR_BGR2HSV)
        lowerValues = np.array([0,150,100])
        higherValues = np.array([50,255,255])
        ret,thres = cv2.threshold(frameC[:,:,2],self.threshold,255,cv2.THRESH_BINARY)
        thres = cv2.inRange(hsv,lowerValues,higherValues)
        #thres = cv2.bitwise_and()
        print('Threshold HSV : '+str(time.time()-t0))
        t0 = time.time()
        ret,thres2 = cv2.threshold(frameC[:,:,2],self.threshold,255,cv2.THRESH_BINARY_INV)
        print('Threshold Bin : '+str(time.time()-t0))
        #thres = cv2.multiply(thres,thres2)
        #kernel = np.ones((5,5),np.uint8)
        #thres = cv2.erode(thres,kernel,iterations = 1)
        #thres = cv2.dilate(thres,kernel,iterations = 3)
        t0 = time.time()
        im2,contours,hierarchy = cv2.findContours(thres ,cv2.RETR_LIST ,cv2.CHAIN_APPROX_SIMPLE)
        print('Contours      : '+str(time.time()-t0))
        cv2.drawContours(frameC,contours,-1,(0,0,255),2)
        
        cv2.circle(frameC,((self.xCrop[-1]),self.xCrop[5]),(self.xCrop[-1]),(0,255,0),3)
        t0 = time.time()
        for contour in contours:
            
            if len(contour)>20:
                print('contour')
                contX = ((contour[:,0,0]-self.xCrop[-1])/(self.xCrop[-1]))
                contY = ((contour[:,0,1]-self.xCrop[5])/(self.xCrop[-1]))
                #print('contour' + str(np.power(contX,2)+np.power(contY,2)))
                phi = np.arctan2(contX,np.sqrt(1-(np.power(contX,2)+np.power(contY,2))))
                theta = np.arctan2(contY,np.sqrt(1-(np.power(contY,2))))
                phiMax = np.max(phi)
                phiMin = np.min(phi)
                iMax = phi.argmax()
                iMin = phi.argmin()
                #print('phi : '+str(phi))
                ptMax1 = (contour[iMax,0,0],contour[iMax,0,1])
                ptMin1 = (contour[iMin,0,0],contour[iMin,0,1])
                ptMax2 = (contour[iMax,0,0],contour[iMax,0,1]+50)
                ptMin2 = (contour[iMin,0,0],contour[iMin,0,1]+50)
                cv2.putText(frameC,"{0:.2f}".format(phiMin/pi2),ptMax1,cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),thickness = 2)
                cv2.putText(frameC,"{0:.2f}".format(phiMin/pi2),ptMin1,cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,128),thickness = 2)
                #cv2.putText(frameC,"{0:.2f}".format(theta[iMax]/pi2),ptMax2,cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),thickness = 2)
                #cv2.putText(frameC,"{0:.2f}".format(theta[iMin]/pi2),ptMin2,cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,128),thickness = 2)
                self.duV = self.duV + (math.sin(phiMax)-math.sin(phiMin))
                self.dpV = self.dpV - (math.cos(phiMax)-math.cos(phiMin))
                self.dudV = self.dudV + (math.sin(phiMax)+math.sin(phiMin))
                self.dpdV = self.dpdV + (math.cos(phiMax)+math.cos(phiMin))
        print('Analysis      : '+str(time.time()-t0))

        #cv2.imshow('anal',thres)
        cv2.imshow('frame',frameC)
        cv2.waitKey(1)
        #cv2.imwrite('./imTest/image'+str(self.i)+'.jpg',frameC)
        #cv2.imwrite('./imTest/thres'+str(self.i)+'.jpg',thres)
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

            camera.resolution = (1280,976)
            #camera.zoom = ( .3563 , 0.2875 , 228/640 , 228/480 )
            camera.framerate = 30
            camera.iso = 800
            camera.shutter_speed = 100000
            #camera.awb_mode = 'on'
            #camera.awb_gains=(1,1)
            #camera.exposure_speed = 100
            #camera.exposure_mode = 'night'
            #camera.exposure_compensation = 25
        
            firstRound = False
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

                            camera.awb_gains=(1, 1)
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
#    if len(sys.argv) > 2:
#        metalog = MetaLog( filename=sys.argv[2] )
#    if len(sys.argv) > 3 and sys.argv[3] == 'F':
#        disableAsserts()


    def TakeOff(self):
        if self.droneConnected:
            self.drone.takeoff()
            self.drone.flyToAltitude(zFlight)

    def emergencyLanding(self):
        if self.droneConnected:
            self.drone.emergency()

    def landing(self):
        if self.droneConnected:
            self.drone.land()


    def move(self):
        roll = 0
        pitch = 0
        yaw = 60
        gaz = 0
        if self.droneConnected:
            drone.update(cmd=movePCMDCmd(active=True, roll=roll, pitch=pitch, yaw=yaw, gaz=gaz))


    def updateVision(self,Vu,Vp,dVu,dVp):
        self.Vu = Vu
        self.Vp = Vp
        self.dVu = dVu
        self.dVp = dVp


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


