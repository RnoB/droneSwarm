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

from bebop import Bebop
from commands import movePCMDCmd






class visionAnalyzer(PiRGBAnalysis):
    duV = 0
    dudV = 0
    dpV = 0
    dpdV = 0
    
    i=0
    threshold = 17

    xCrop = []
    t0 = 0
    def __init__(self,camera,sectionCrop):
        super(visionAnalyzer,self).__init__(camera)
        self.xCrop = sectionCrop
        

    def analyze(self,frame):

        self.duV = 0
        self.dudV = 0
        self.dpV = 0
        self.dpdV = 0
        #self.threshold=self.threshold+1
        #if self.threshold>200:
        #    self.threshold = 0
        frameC = frame[:,self.xCrop[0]:self.xCrop[1],:]
        ret,thres = cv2.threshold(frameC[:,:,0],self.threshold,255,cv2.THRESH_BINARY)
        ret,thres2 = cv2.threshold(frameC[:,:,2],self.threshold,255,cv2.THRESH_BINARY_INV)
        thres = cv2.multiply(thres,thres2)
        #kernel = np.ones((5,5),np.uint8)
        #thres = cv2.erode(thres,kernel,iterations = 1)
        #thres = cv2.dilate(thres,kernel,iterations = 3)
        im2,contours,hierarchy = cv2.findContours(thres ,cv2.RETR_LIST ,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frameC,contours,-1,(0,0,255),2)


        for contour in contours:
            if len(contour)>20:
                contCircleX = ((contour[:,0,1]-self.xCrop[-1])/(2.*self.xCrop[-1]))*math.pi
                theta = ((contour[:,0,1]-self.xCrop[-1])/(2.*self.xCrop[-1]))*math.pi

                phi = (np.sqrt(np.power(contCircleX,2)+np.power(theta,2))-math.pi/2.)*np.cos(theta)
                    
                
                phiMax = np.max(phi)
                phiMin = np.min(phi)
                self.duV = self.duV - (math.sin(phiMax)-math.sin(phiMin))
                self.dpV = self.dpV - (math.cos(phiMax)-math.cos(phiMin))
                self.dudV = self.dudV + (math.sin(phiMax)+math.sin(phiMin))
                self.dpdV = self.dpdV + (math.cos(phiMax)+math.cos(phiMin))

            
        cv2.imwrite('./imTest/image'+str(self.i)+'.jpg',frameC)
        cv2.imwrite('./imTest/thres'+str(self.i)+'.jpg',thres)
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
        xMin = crop[0]
        yMin = crop[1]
        RMax = crop[2]
        xMax = xMin+2*RMax
        yMax = yMin+2*RMax
        xCenter = xMin+RMax
        yCenter = yMin+RMax
        self.xCrop = [xMin,xMax,yMin,yMax,xCenter,yCenter,RMax]

    def visionUpdater(self):

        with picamera.PiCamera() as camera:

            camera.resolution = (1296,976)
            #camera.zoom = ( .3563 , 0.2875 , 228/640 , 228/480 )
            camera.framerate = 10
            camera.iso = 1600
            camera.shutter_speed = 50000
            #camera.awb_mode = 'off'
            #camera.awb_gains=(1,1)
            #camera.exposure_speed = 100
            #camera.exposure_mode = 'night'
            camera.exposure_compensation = 25
        
            firstRound = True
            running =True 
    
            try:
                k = 0

                with visionAnalyzer(camera,self.xCrop) as anal:
                    camera.start_recording(anal, 'rgb')
                #for frame in enumerate(camera.capture_continuous(rawCapture, 'rgb')):#,resize=(228,228))):
                    while running:
                        
                        k=k+1
                        #print('analog gain : '+str(camera.analog_gain)+' digital_gain : '+str(camera.digital_gain))
                        camera.wait_recording(1.0/camera.framerate)
                        if camera.analog_gain >5 and camera.digital_gain > 1 and firstRound:
                            
                            camera.exposure_mode = 'off'
                            camera.awb_mode = 'off'
                            b=.5
                            r=0.0

                            camera.awb_gains=(5,6)
                            firstRound = False
                            
                        else:
                            self.Vu =anal.duV
                            self.Vp =anal.dpV
                            self.dVu=anal.dudV
                            self.dVp=anal.dpdV 
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
            self.drone.landing()


    def move(self):
        roll = 0
        pitch = 0
        yaw = 0
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
            self.drone = Bebop()
            self.drone.trim()
            self.droneConnected = True
        except:
            self.droneConnected = False
            


    def __init__(self):
        self.zFlight = 0.5
        metalog=None
        print('Drone Control is intialized -- Ready to start')


