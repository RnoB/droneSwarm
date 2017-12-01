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




class visionAnalyzer(PiRGBAnalysis):
    duV = 0
    dudV = 0
    dpV = 0
    dpdV = 0
    
    i=0
    threshold = 150

    xCrop = []
    t0 = 0

    fgbg = cv2.createBackgroundSubtractorMOG()

    def __init__(self,camera):
        super(visionAnalyzer,self).__init__(camera)
        self.xCrop = sectionCrop
        

    def analyze(self,frame):



        fgmask = self.fgbg.apply(frame)
        cv2.imshow('frame',fgmask)
        cv2.waitKeys(1)
        #self.threshold=self.threshold+1
        #if self.threshold>200:
        #    self.threshold = 0

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
            camera.iso = 800
            camera.shutter_speed = 100000
            camera.awb_mode = 'off'
            camera.awb_gains=(8,8)
            #camera.exposure_speed = 100
            #camera.exposure_mode = 'night'
            camera.exposure_compensation = 25
        
            firstRound = True
            running =True 
    
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

