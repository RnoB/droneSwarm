import numpy as np
import threading
import time
import socket
import swarmNet
import os
import math
from colorsys import hsv_to_rgb as hsv2rgb
from sense_hat import SenseHat



black = (0,0,0)
red = (255,0,0)
green = (0,255,0)
blue = (0,0,255)
cyan = (0,255,255)


class senseController():
    status = [0,0,0,0,0,0,0,0]
    sense = SenseHat()
    running = True
    step = 0
    def greenMat(self):
        for k in range(0,8):
            for j in range(4,8):
                self.sense.set_pixel(k,j,green)

    def blueMat(self):
        for k in range(0,8):
            for j in range(4,8):
                self.sense.set_pixel(k,j,blue)


    def cyanMat(self):
        for k in range(0,8):
            for j in range(4,8):
                self.sense.set_pixel(k,j,cyan)

    def blackMat(self):
        for k in range(0,8):
            for j in range(0,8):
                self.sense.set_pixel(k,j,black)


    def pushed_right(self):
        
        self.step=1


    def pushed_up(self):
        
        if self.step==0:
            self.step = 2
            self.blueMat()
        if self.step == 2:
            self.step = 0
            self.greenMat()


    def pushed_left(self):
        self.step = 2


    def pushed_down(self):
        
        for k in range(0,len(swarmNet.dronesIP)):
            self.status[k] = swarmNet.droneComm(swarmNet.dronesIP[k],dronePort = swarmNet.droneCommRightPort)
            if self.status[k] == 0:
                self.sense.set_pixel(k,0,blue)
                self.sense.set_pixel(k,1,blue)
            elif self.status[k] == 1:
                self.sense.set_pixel(k,0,cyan)
                self.sense.set_pixel(k,1,blue)
            elif self.status[k] == 2:
                self.sense.set_pixel(k,0,cyan)
                self.sense.set_pixel(k,1,green)


    def pushed_middle(self):
        
        self.cyanMat()
        if self.step == 0:
            self.step = 1
            for k in range(0,len(swarmNet.dronesIP)):
                if self.status[k] == 2:
                    try:
                        swarmNet.droneComm(swarmNet.dronesIP[k],code = swarmNet.startCode[0],dronePort = swarmNet.droneCommRightPort)
                    except:
                        pass
            self.step = 2
            self.blueMat()
        elif self.step == 2:
            self.step = 1
            for k in range(0,len(swarmNet.dronesIP)):
                if self.status[k] == 2:
                    try:
                        swarmNet.droneComm(swarmNet.dronesIP[k],code = swarmNet.startCode[2], dronePort = swarmNet.droneCommRightPort)
                    except:
                        pass
            self.step = 0
            self.greenMat()


    def refresh(self):
        self.sense.clear()

        self.blackMat()

    def __init__(self):
        
        self.refresh()
        
        self.greenMat()
        self.pushed_down()
        self.sense.set_imu_config(True, True, True)
        self.sense.stick.direction_right = self.pushed_up
        self.sense.stick.direction_up = self.pushed_left
        self.sense.stick.direction_left = self.pushed_down
        self.sense.stick.direction_down = self.pushed_right
        self.sense.stick.direction_middle = self.pushed_middle
        

        t0 = time.time()
        ti=t0
        #displayStart()

        #while self.running:
        #    time.sleep(60)
        #    print('---- waiting for your drones')
        #    self.pushed_down()
            








if __name__ == '__main__':
    main()