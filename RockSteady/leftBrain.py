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

tStatus = 0
tStatusEmergency = 15

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
    global tStatus
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
        #print('--- waiting for a connection')
        try:
            connection, client_address = server.accept()
            #print('------ Connection coming from ' + str(client_address))



            code = struct.unpack('i',connection.recv(4))[0]
            #print('------ code : '+ str(code))
            if code == swarmNet.requestStatusCode:
                tStatus = 0
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
                #droneController.start()
                droneController.TakeOff()
                started = True


            if code == swarmNet.startCode[2]:
                data = struct.pack('i', code+1)

                
                try:
                    droneController.landing()
                except:
                    droneController.emergencylanding()
            if code == swarmNet.updateVisionCode[0]:
                data= struct.unpack('dddd',connection.recv(32))
                print('the right world looks like : '+str(data))
                VuR = data[0]
                VpR = data[1]
                dVuR = data[2]
                dVpR = data[3]

                data = struct.pack('i', code+1)
                try:
                    connection.sendall(data)
                except:
                    print('sending did not work :/ but better not break everything')
            if code == swarmNet.updateParameters[0]:
                data= struct.unpack('dddddddd',connection.recv(64))
                print('new parameters : '+str(data))
                droneController.updateParameters(au=data[0],ru=data[1],ap=data[2],rp=data[3],um=data[4],pm=data[5],v0=data[6],a=data[7])
                data = struct.pack('i', code+1)
                try:
                    connection.sendall(data)
                except:
                    print('sending did not work :/ but better not break everything')

        except ValueError:
            print(ValueError)




            


def droneControl():
    

    global started
    tSleep = .2
    while running:
        time.sleep(tSleep)
        if started:
            
            
            droneController.updateVision(vision.Vu-VuR,vision.Vp-VpR,vision.dVu-dVuR,vision.dVp-dVp)
            droneController.move()



def main():
    global running
    global state
    global droneController
    global droneConnected
    global started
    global tStatus
    t0 = time.time()
    time.sleep(30)
    ti=t0
    state = 1
    #displayStart()
    print('the left brain IP is : '+str(swarmNet.leftBrainIP))
    statusThread = threading.Thread(target = droneCommServer, args=(swarmNet.leftBrainIP,))
    statusThread.daemon = True
    statusThread.start()
    statusThread = threading.Thread(target = droneControl, args=())
    statusThread.daemon = True
    statusThread.start()
    try:
        eyeProp = np.loadtxt('/home/pi/droneSpecs.csv')
        vision.sectionCrop(eyeProp)
        vision.start()
        started = True
        droneController.start()
        droneController.updateParameters()
        droneConnected =True
    except:
        started=True
        droneConnected =False

    while running:

        
        time.sleep(5)
        print('---- waiting for your drones')
        tStatus = tStatus+5
        if tStatus>tStatusEmergency:
            try:
            
                print('lobotomy !!! ')
                droneController.landing()

            except:
                droneController.emergencyLanding()

            state = 1        








if __name__ == '__main__':
    main()