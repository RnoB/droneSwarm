import numpy as np
import threading
import time
import socket
import swarmNet
import os
import math
import senseControl

def krangServer(ip):
    print(' - - -- starting the drone center command')
    global running
    global started
    global state

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

                try:
                    connection.sendall(data)
                except:
                    print('sending did not work :/ but better not break everything')
        except ValueError:
            print(ValueError)

running = True
def main():
    global running

    senseController = senseControl.senseController()
    t0 = time.time()
    ti=t0
    #displayStart()
    statusThread = threading.Thread(target = swarmNet.giveStatus, args=(swarmNet.krangIP,))
    statusThread.daemon = True
    statusThread.start()
    statusThread = threading.Thread(target = krangServer, args=(swarmNet.krangIP,))
    statusThread.daemon = True
    statusThread.start()

    while running:
        time.sleep(60)
        print('---- waiting for your drones')
        








if __name__ == '__main__':
    main()