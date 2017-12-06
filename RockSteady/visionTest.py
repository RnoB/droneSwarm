import numpy as np
import threading
import time
import socket
import swarmNet
import os
import math
import droneControl
import struct


state = 0
rightState = state
running = True
started = False
vision = droneControl.vision()


def main():
    global running
    global started
    t0 = time.time()
    time.sleep(30)
    ti=t0
    state = 1
    #displayStart()
    droneId = int(next(open('/home/pi/droneId')))-1
    print('the drone Ip is : '+str(swarmNet.dronesIP[droneId]))
    print('the tight brain IP is : '+str(swarmNet.rightBrainIP))

    try:
        eyeProp = np.loadtxt('/home/pi/droneSpecs.csv')
        vision.sectionCrop(eyeProp)
        vision.start()
        started = True
    except ValueError:
        print(ValueError)
    while running:

        
        time.sleep(60)
        print('---- waiting for your drones')
        








if __name__ == '__main__':
    main()