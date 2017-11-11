import numpy as np
import threading
import time
import socket
import swarmNet
import os
import math
from colorsys import hsv_to_rgb as hsv2rgb
from sense_hat import SenseHat




class senseController():

    sense = SenseHat()
    running = True
    step = 0

    pixB=[]
    jk=np.ones(8)
    for j in range(0,8):
        for k in range(0,8):
            pixB.append(hsv2rgb(0,0,0))
    def pushed_right(self):
        self.step=1
        print('right')


    def pushed_up(self):
        self.step=0
        print('up')

    def pushed_left(self):
        self.step=2
        print('left')


    def pushed_down(self):
        self.step=3
        print('down')

    def pushed_middle(self):
        self.step=3
        print('middle')


    def refresh(self):
        self.sense.clear()

        self.sense.set_pixels(self.pixB)

    def __init__(self):
        
        self.refresh()
        pix1=[]
        jk=np.ones(8)
        for j in range(0,8):
            for k in range(0,8):
                col = hsv2rgb(k*j/49,1,1)
                print('color : '+str(col))
                col = tuple(int(255*x) for x in col)
                print('color : '+str(col))
                pix1.append(col)
        print(pix1)
        self.sense.set_pixels(pix1)
        self.sense.set_imu_config(True, True, True)
        self.sense.stick.direction_right = self.pushed_right
        self.sense.stick.direction_up = self.pushed_up
        self.sense.stick.direction_left = self.pushed_left
        self.sense.stick.direction_down = self.pushed_down
        self.sense.stick.direction_middle = self.pushed_middle
        self.sense.stick.direction_any = self.refresh

        t0 = time.time()
        ti=t0
        #displayStart()

        while self.running:
            time.sleep(60)
            print('---- waiting for your drones')
            self.refresh()
            








if __name__ == '__main__':
    main()