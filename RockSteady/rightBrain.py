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
Vu = 0
Vp = 0
dVu = 0
dVp = 0
vision = droneControl.vision()

def droneCommServer(ip):
    print(' - - -- starting the drone center command')
    global running
    global started
    global state
    backlog = 1  # how many connections to accept
    maxsize = 28
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    binded = False
    while not binded:
        try:
            server.bind((ip,swarmNet.droneCommRightPort))
            binded = True
        except:
            print('-Control Center -- binding failed')
            binded = False
            time.sleep(5)
    server.listen(1)
    while running:
        print('--- waiting for a connection')
        try:
            connection, client_address = server.accept()
            print('------ Connection coming from ' + str(client_address))



            code = struct.unpack('i',connection.recv(4))[0]
            print('------ code : '+ str(code))
            if code == swarmNet.requestStatusCode:
                print('state : '+str(state))
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
                started = True
                swarmNet.droneComm(swarmNet.leftBrainIP,code = code)
                
            if code == swarmNet.startCode[2]:
                data = struct.pack('i', code+1)
                try:
                    connection.sendall(data)
                except:
                    print('sending did not work :/ but better not break everything')
                started = False
                swarmNet.droneComm(swarmNet.leftBrainIP,code = code)
            if code == swarmNet.updateParameters[0]:
                dataSend = struct.unpack('dddddddd',connection.recv(64))
                data = struct.pack('i', code+1)
                try:
                    connection.sendall(data)
                except:
                    print('sending did not work :/ but better not break everything')
                started = False
                swarmNet.droneComm(swarmNet.leftBrainIP,code = code,dataSend=dataSend)
                
        except:
            pass




def brainStatus(IP):
    global state
    tSleep = 10
    while running:
        print('--- Check Brain Connectivity')
        time.sleep(tSleep)
        try:
            leftState = swarmNet.droneComm(IP)
            print('lobotomy ? '+str(leftState ))
            if leftState == 1:
                state = 2
            else:
                state = 1
        except:
            print('lobotomy !!! ')
            state = 1



def droneControl():
    
    global started
    tSleep = .3
    while running:
        time.sleep(tSleep)
        if started:
            #print('sending the right world')
            #print('with the following vision : '+str([vision.Vu,vision.Vp,vision.dVu,vision.dVp]))
            swarmNet.droneComm(swarmNet.leftBrainIP,code=swarmNet.updateVisionCode[0],dataSend=[vision.Vu,vision.Vp,vision.dVu,vision.dVp])


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
    statusThread = threading.Thread(target = droneCommServer, args=(swarmNet.dronesIP[droneId],))
    statusThread.daemon = True
    statusThread.start()
    statusThread = threading.Thread(target = swarmNet.giveStatus, args=(swarmNet.rightBrainIP,))
    statusThread.daemon = True
    statusThread.start()
    statusThread = threading.Thread(target = brainStatus, args=(swarmNet.leftBrainIP,))
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
    except ValueError:
        print(ValueError)
    while running:

        
        time.sleep(60)
        print('---- waiting for your drones')
        








if __name__ == '__main__':
    main()