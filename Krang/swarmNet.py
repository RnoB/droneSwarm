import numpy
import threading
import time
import socket
import struct
import numpy as np
import uuid

krangIP = '192.168.30.1'

dronesIP = ['192.168.30.11', '192.168.30.12', '192.168.30.13',
            '192.168.30.14', '192.168.30.15']


broadcastIP = '192.168.30.255'


rightBrainIP = '192.168.40.1'
leftBrainIP = '192.168.40.2'

droneIP = ''


statusPort = 5007
droneCommRightPort = 5108
droneCommLeftPort = 5208


requestStatusCode = 1001
sendStatusCode = 1002

startCode = [2001,2002,2003,2004]
updateVisionCode = [3001,3002]
updateParameters = [4001,4002]
killCode = 99999

running = True


max_int64 = 0xFFFFFFFFFFFFFFFF


def broadcastSize(fishVR,size):
    global running
    socketSend = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    socketSend.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    socketSend.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    print('>> Apoc wants everybody to know the size of the fish : ')


    data = struct.pack('id', fishVR,size)

    #print('Send : ' + str(struct.unpack('dddi', data)))
    try:
        socketSend.sendto(data, (apocWifi , measurementPort + 20))
        time.sleep(1)
    except:
        print('sometthing went with wrong  broadcasting')

    socketSend.close()
    print('Sender Disconnected')

def giveStatus(ip):
    backlog = 1  # how many connections to accept
    maxsize = 28
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    binded = False
    while not binded:
        try:
            server.bind((ip,statusPort))
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
            if code == requestStatusCode:
                data = struct.pack('i', sendStatusCode)
                try:
                    connection.sendall(data)
                except:
                    print('sending did not work :/ but better not break everything')
        except:
            pass


def requestStatus(ip):
    status = False
    socketClient = socket.socket()
    socketClient.settimeout(1)
    socketClient.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    connect = 0
    fishN=0
    print('--- Connection to ' + str(ip))

    try:
        socketClient.connect((ip, statusPort))
        print('--- Connected to ' + str(ip))
        try:
            data = struct.pack('i', requestStatusCode)
            socketClient.sendall(data)
            code = struct.unpack('ii',socketClient.recv(8))[0]
            if code[0] == sendStatusCode:
                status = code[1]      
        except:
            status = 0
        finally:
            socketClient.shutdown(socket.SHUT_RDWR)
            socketClient.close()


    except:
        pass
        #print('--- connection failed')  
    return status






def droneComm(ip,dronePort = droneCommLeftPort,code = requestStatusCode,dataSend = []):
    status = 0
    socketClient = socket.socket()
    socketClient.settimeout(1)
    socketClient.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    connect = 0
    fishN=0
    print('--- Connection to ' + str(ip))

    try:
        socketClient.connect((ip, dronePort))
        print('--- Connected to ' + str(ip))
        print('Sending the code : '+str(code))
        try:
            if code == requestStatusCode:
                data = struct.pack('i', code)
                socketClient.sendall(data)
                dataRec = struct.unpack('ii',socketClient.recv(8))
                print(dataRec)
                code =dataRec[0]
                status = dataRec[1]
            if code in startCode:
                data = struct.pack('i', code)
                socketClient.sendall(data)
                dataRec = struct.unpack('i',socketClient.recv(4))
                status =dataRec[0]
            if code in updateVisionCode:
                data = struct.pack('i', code)
                socketClient.sendall(data)
                data = struct.pack('dddd', dataSend[0],dataSend[1],dataSend[2],dataSend[3])
                socketClient.sendall(data)
                dataRec = struct.unpack('i',socketClient.recv(4))
                status =dataRec[0]
            if code in updateParameters:
                data = struct.pack('i', code)
                socketClient.sendall(data)
                data = struct.pack('dddddddd', dataSend[0],dataSend[1],dataSend[2],dataSend[3],dataSend[4],dataSend[5],dataSend[6],dataSend[7])
                socketClient.sendall(data)
                dataRec = struct.unpack('i',socketClient.recv(4))
                status =dataRec[0]
        
        finally:
            socketClient.shutdown(socket.SHUT_RDWR)
            socketClient.close()

    except:
        print('--- connection failed')
        pass  
    return status
