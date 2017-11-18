import numpy as np
import threading
import time
import socket
import swarmNet
import os
import math
import droneSpecs
import droneControl
import struct

state = 0
running = True
started = False
droneConnected =False
Vu = 0
Vp = 0
dVu = 0
dVp = 0
VuR = 0
VpR = 0
dVuR = 0
dVpR = 0
vision = droneControl.vision()
droneController = droneControl.droneController()


def droneCommServer(ip):
    print(' - - -- starting the drone center command')
    global running
    global started
    global state
    global VuR
    global VpR
    global dVuR
    global dVpR
    global started
    backlog = 1  # how many connections to accept
    maxsize = 28
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    binded = False
    while not binded:
        try:
            server.bind((ip,swarmNet.droneCommLeftPort))
            binded = True
        except:
            print('- Give Status -- binding failed')
            binded = False
            time.sleep(20)
    server.listen(1)
    while running:
        print('--- waiting for a connection')
        try:
            connection, client_address = server.accept()
            print('------ Connection coming from ' + str(client_address))



            code = struct.unpack('i',connection.recv(4))[0]
            print('------ code : '+ str(code))
            if code == swarmNet.requestStatusCode:
                data = struct.pack('ii', swarmNet.sendStatusCode,state)
                try:
                    connection.sendall(data)
                except:
                    print('sending did not work :/ but better not break everything')
            if code == swarmNet.startCode[0]:
                data = struct.pack('i', code+1)
                try:
                    connection.sendall(data)
                except:
                    print('sending did not work :/ but better not break everything')
                
                droneControl.TakeOff()
                started = True

                state =2
            if code == swarmNet.startCode[2]:
                data = struct.pack('i', code+1)
                try:
                    connection.sendall(data)
                except:
                    print('sending did not work :/ but better not break everything')
                state = 1
                swarmNet.droneComm(swarmNet.leftBrainIP,code)
                try:
                    droneController.landing()
                except:
                    droneController.emergencylanding()
            if code == swarmNet.updateVisionCode[0]:
                data= struct.unpack('dddd',connection.recv(32))
                #print('the right world looks like : '+str(data))
                VuR = data[0]
                VpR = data[1]
                dVuR = data[2]
                dVpR = data[3]

                data = struct.pack('i', code+1)
                try:
                    connection.sendall(data)
                except:
                    print('sending did not work :/ but better not break everything')
        except ValueError:
            print(ValueError)



def brainStatus(IP):
    global state
    tSleep = 10

    while running:
        print('--- Check Brain Connectivity')
        time.sleep(tSleep)
        try:
            lobotomyState = swarmNet.requestStatus(IP)
            print('not lobotomy ? '+str(lobotomyState))
            if not lobotomyState:
                print('lobotomy !!! ')
                droneController.emergencyLanding()
                state = 0

        except:
            print('lobotomy !!! ')
            droneController.emergencyLanding()
            


def droneControl():
    

    global started
    tSleep = .01
    while running:
        time.sleep(tSleep)
        if started:
            
            
            droneController.updateVision(vision.Vu+VuR,vision.Vp+VpR,vision.dVu+dVuR,vision.dVp+dVp)
            droneController.move()



def main():
    global running
    global state
    global droneController
    global droneConnected
    global started
    t0 = time.time()
    ti=t0
    state = 1
    #displayStart()
    # print('the left brain IP is : '+str(swarmNet.leftBrainIP))
    # statusThread = threading.Thread(target = droneCommServer, args=(swarmNet.leftBrainIP,))
    # statusThread.daemon = True
    # statusThread.start()
    # statusThread = threading.Thread(target = brainStatus, args=(swarmNet.rightBrainIP,))
    # statusThread.daemon = True
    # statusThread.start()
    # statusThread = threading.Thread(target = droneControl, args=())
    # statusThread.daemon = True
    # statusThread.start()
    try:

        # vision.sectionCrop(droneSpecs.leftEye)
        # vision.start()
        started = True
        droneController.start()
        droneConnected =True
    except:
        started=True
        droneConnected =False

    # while running:

        
    #     time.sleep(60)
    #     print('---- waiting for your drones')
        








if __name__ == '__main__':
    main()