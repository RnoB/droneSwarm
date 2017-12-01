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
import csv
import os




class visionAnalyzer(PiRGBAnalysis):
    duV = 0
    dudV = 0
    dpV = 0
    dpdV = 0
    
    i=0
    threshold = 150

    xCrop = []
    t0 = 0
    thres2 = []

    circleStep = 0
    radiusStep = 0
    center = (0,0)
    radius = 0
    def __init__(self,camera):
        super(visionAnalyzer,self).__init__(camera)
        
        

    def analyze(self,frame):


        
        #v2.namedWindow('frame',cv2.WINDOW_NORMAL)
        #cv2.resizeWindow('frame',1280,976)
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        ret,thres = cv2.threshold(hsv[:,:,2],self.threshold,255,cv2.THRESH_BINARY)
        if self.circleStep == 0:
            self.thres2 = thres
        else:
            self.thres2 = thres + self.thres2
        #cv2.imshow('frame',self.thres2)
        #self.threshold=self.threshold+1
        #if self.threshold>200:
        #    self.threshold = 0

        im2,contours,hierarchy = cv2.findContours(self.thres2 ,cv2.RETR_LIST ,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame,contours,-1,(0,0,255),2)

        for contour in contours:
            if len(contour)>1000:
                (x,y),radius = cv2.minEnclosingCircle(contour)
                center = (int(x),int(y))
                radius = int(radius)
                print(center)
                print(radius)
                cv2.circle(frame,center,radius,(255,0,0),2)
                if self.center == center and self.radius == radius:
                    self.radiusStep = self.radiusStep + 1
                else:
                    self.radiusStep = 0
                self.center = center
                self.radius = radius
        self.circleStep = self.circleStep+1
        #cv2.imshow('frame',self.thres2)
        #cv2.imshow('frame',frame)
        #cv2.waitKey(1)
        if self.radiusStep == 20:
            
            
            rad = np.asarray([self.radius,self.center[0],self.center[1]])
            np.savetxt('droneSpecs.csv',rad,delimiter=",")
            
            
            os.system('killall python3')
        



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

            camera.resolution = (1280,976)
            #camera.zoom = ( .3563 , 0.2875 , 228/640 , 228/480 )
            camera.framerate = 30
            camera.iso = 400
            camera.shutter_speed = 2000
            #camera.awb_mode = 'off'
            #camera.awb_gains=(8,8)
            #camera.exposure_speed = 100
            #camera.exposure_mode = 'night'
            #camera.exposure_compensation = 25
        
            firstRound = False
            running =True 
            print('starting')
            try:
                k = 0

                with visionAnalyzer(camera) as anal:
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

