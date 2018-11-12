import time
import picamera
import picamera.array
from picamera.array import PiRGBAnalysis
import cv2
from gpiozero import Button
import threading
import sys
import random
import math
import numpy as np
import traceback

capture =False
running = True
analyze = False
text = False
cv2.namedWindow("window_name", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("window_name", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

button1 = Button(18)
button2 = Button(23)
button3 = Button(24)
button4 = Button(21)

modes = [0,1,2]
modeSwitch = 0
mode = modes[modeSwitch]

xCrop = []

font = cv2.FONT_HERSHEY_SIMPLEX
def generateSinFunction():


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
    R2=np.copy(R)
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
    newVision = False
    i=0
    thresholdRed = 20
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


        


    def __init__(self,camera,section):
        super(visionAnalyzer,self).__init__(camera)
        self.xCrop = section

        print('generateSinFunction')
        self.VcoscosA,self.VcoscosR,self.VcossinA,self.VcossinR,self.VsinA,self.VsinR,self.circle=generateSinFunction()
        print('generatedSinFunction')


        

    def analyze(self,frame):
        global capture
        frameC = frame[:,self.xCrop[0]:self.xCrop[1],:]
        #t0=time.time()
        if not analyze:
            cv2.imshow("window_name", frameC)
            cv2.waitKey(1)
            if capture:
                timestr = time.strftime("%Y%m%d-%H%M%S")
                cv2.imwrite('/home/pi/Pictures/Blue'+timestr+'.jpg',frameC)
                capture = False
        else:
            maskRB = np.subtract(frameC[:,:,2], frameC[:,:,1].astype(np.int16))*self.circle
            maskRB=maskRB>self.thresholdRed
            maskdRB = (np.roll(maskRB,1,axis=1) != np.roll(maskRB,-1,axis=1))
            
            frameD = cv2.bitwise_and(frameC,frameC,mask = maskRB.astype(np.uint8))
            n = str(self.i).zfill(5)
            self.duV = np.sum(self.VcoscosR[maskRB])
            self.dudV = np.sum(self.VcoscosA[maskdRB])/2.0
            self.dpV = np.sum(self.VcossinR[maskRB])
            self.dpdV = np.sum(self.VcossinA[maskdRB])/2.0
            if text:
                cv2.putText(frameD,"%10.4f"%(self.duV),(10,30),font,1,(0,255,0),2)
                cv2.putText(frameD,"%10.4f"%(self.dudV),(10,60),font,1,(0,255,0),2)
                cv2.putText(frameD,"%10.4f"%(self.dpV),(10,90),font,1,(0,255,0),2)
                cv2.putText(frameD,"%10.4f"%(self.dpdV),(10,120),font,1,(0,255,0),2)
            cv2.imshow("window_name", frameD)
            cv2.waitKey(1)
            if capture:
                timestr = time.strftime("%Y%m%d-%H%M%S")
                cv2.imwrite('/home/pi/Pictures/Blue'+timestr+'.jpg',frameC)
                cv2.imwrite('/home/pi/Pictures/Cut'+timestr+'.jpg',frameD)
                cv2.imwrite('/home/pi/Pictures/Thres'+timestr+'.jpg',maskRB.astype(np.uint8)*255)
                capture = False

            self.i=self.i+1
            t1=time.time()
            self.t0 = t1
            self.newVision = True
            time.sleep(.01)



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

def buttonLogger():
    global capture
    global analyze
    global modeSwitch
    global mode
    global text
    while True:
        #print("1 : "+str(button1.is_pressed))
        #print("2 : "+str(button2.is_pressed))
        #print("3 : "+str(button3.is_pressed))
        if button4.is_pressed:
            capture = True
        if button1.is_pressed:
            modeSwitch = (modeSwitch+1)%len(modes)
            mode = modes[modeSwitch]
            print(mode)
            button1.wait_for_release(1)
        if button2.is_pressed:
            analyze = not analyze
            button2.wait_for_release(1)
        if button3.is_pressed:
            text = not text
            button3.wait_for_release(1)

        time.sleep(.1)

def captureImage():
    global capture
    with picamera.PiCamera() as camera:
        camera.resolution = (2592, 1944)
        camera.start_preview()
        # Camera warm-up time
        time.sleep(2)
        timestr = time.strftime("%Y%m%d-%H%M%S")
        camera.capture('/home/pi/Pictures/image'+timestr+'.jpg')
        capture = False

def streamCamera():
    with picamera.PiCamera() as camera:
        camera.resolution = (320, 240)
        camera.framerate = 30
        camera.led = False
        #camera.start_preview()

        time.sleep(2)
        with picamera.array.PiRGBArray(camera) as stream:
            while not capture and mode == 0:
                camera.capture(stream, format='bgr', use_video_port=True)

                # At this point the image is available as stream.array
                frame = stream.array
                cv2.imshow("window_name", frame)
                cv2.waitKey(1)
                stream.seek(0)
                stream.truncate()


def cameraDrone():
    with picamera.PiCamera() as camera:

        camera.resolution = (1296,976)
        #camera.zoom = ( .3563 , 0.2875 , 228/640 , 228/480 )
        camera.framerate = 3
        camera.iso = 800
        camera.shutter_speed = 50000
        camera.awb_mode = 'off'
        camera.awb_gains=(2.5,6)
        #camera.exposure_speed = 100
        #camera.exposure_mode = 'night'
        camera.exposure_compensation = 20

        firstRound = True
        running =True 
        print('here')
        try:
            k = 0

            with visionAnalyzer(camera,xCrop) as anal:
                camera.start_recording(anal, 'bgr')
            #for frame in enumerate(camera.capture_continuous(rawCapture, 'rgb')):#,resize=(228,228))):
                while mode == 1:
                    
                    k=k+1
                    #print('analog gain : '+str(camera.analog_gain)+' digital_gain : '+str(camera.digital_gain))
                    camera.wait_recording(1.0/camera.framerate)
                    if camera.analog_gain >7 and camera.digital_gain > 1 and firstRound:
                        
                        camera.exposure_mode = 'off'
                        camera.awb_mode = 'off'
                        b=0
                        r=0

                        camera.awb_gains=(1, 0)
                        firstRound = False
                        print(camera.awb_gains)


                    if not firstRound and False:
                        r=r+.1
                        if r>8:
                            r=0
                            b=b+.1
                        if b>8:
                            break

                        camera.awb_gains=(r,b)
                        print(camera.awb_gains)
                camera.stop_recording()
        except:
            traceback.print_exc()
            


def wbCalib():
    global capture
    with picamera.PiCamera() as camera:

        camera.resolution = (1296,976)
        #camera.zoom = ( .3563 , 0.2875 , 228/640 , 228/480 )
        camera.framerate = 3
        camera.iso = 800
        camera.shutter_speed = 50000
        camera.awb_mode = 'off'
        camera.awb_gains=(2.5,6)
        #camera.exposure_speed = 100
        #camera.exposure_mode = 'night'
        camera.exposure_compensation = 20

        firstRound = True
        firstWB = True
        running =True 
        print('here')
        time.sleep(2)
        stepWB = .3
        with picamera.array.PiRGBArray(camera) as stream:
            while  mode == 2:
                camera.capture(stream, format='bgr', use_video_port=True)

                # At this point the image is available as stream.array
                frame = stream.array
                frame = frame[:,xCrop[0]:xCrop[1],:]
                cv2.imshow("window_name", frame)
                cv2.waitKey(1)
                stream.seek(0)
                stream.truncate()
                if camera.analog_gain >7 and camera.digital_gain > 1 and firstRound:
                    
                    camera.exposure_mode = 'off'
                    camera.awb_mode = 'off'
                    b=0
                    r=0

                    camera.awb_gains=(1, 1)
                    firstRound = False
                    print(camera.awb_gains)
                if capture and not firstRound:
                    if firstWB:
                        timestr = time.strftime("%Y%m%d-%H%M%S")
                        r = 0
                        b = 0
                        camera.awb_gains=(0,0)
                        firstWB = False
                    else:

                        
                        rName = '%.1f' % r
                        bName = '%.1f' % b
                        cv2.imwrite('/home/pi/Pictures/WB/WB-'+timestr+'-r_'+rName+'-b_'+bName+'.jpg',frame)

                        r=r+stepWB
                        if r>8:
                            r=0
                            b=b+stepWB
                        if b>8:
                            capture = False
                            firstWB = True
                            camera.awb_gains=(1,1)
                        else:
                            camera.awb_gains=(r,b)
                        


                    







def main():
    global xCrop
    buttonThread = threading.Thread(target=buttonLogger)
    buttonThread.daemon = True
    buttonThread.start()
    eyeProp = np.loadtxt('/home/pi/droneSpecs.csv')
    xCrop = sectionCrop(eyeProp)
    while running:
        time.sleep(.1)
        if mode == 0:
            if capture:
                captureImage()
            else:
                streamCamera()
        if mode == 1:
            cameraDrone()
        if mode == 2:
            wbCalib()

if __name__ == "__main__":
    main()