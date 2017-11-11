import numpy as np
import threading
import time
import socket
import swarmNet
import os
import math
import senseControl

def main():
    global running

    senseController = senseControl.senseController()
    t0 = time.time()
    ti=t0
    #displayStart()
    statusThread = threading.Thread(target = swarmNet.giveStatus, args=(swarmNet.krangIP,))
    statusThread.daemon = True
    statusThread.start()

    while running:
        refresh()
        time.sleep(60)
        print('---- waiting for your drones')
        








if __name__ == '__main__':
    main()