#!/usr/bin/python
"""
  Basic class for communication to Parrot Bebop
  usage:
       ./bebop.py <task> [<metalog> [<F>]]
"""
import sys
import socket
import datetime
import struct
import time

from navdata import *
from commands import *

# this will be in new separate repository as common library fo robotika Python-powered robots
from apyros.metalog import MetaLog, disableAsserts


# hits from https://github.com/ARDroneSDK3/ARSDKBuildUtils/issues/5

HOST = "192.168.42.1"
DISCOVERY_PORT = 44444

NAVDATA_PORT = 43210 # d2c_port
COMMAND_PORT = 54321 # c2d_port

class Bebop:
    def __init__( self, metalog=None, onlyIFrames=True ):
        if metalog is None:
            self._discovery()
            metalog = MetaLog()
        self.navdata = metalog.createLoggedSocket( "navdata", headerFormat="<BBBI" )
        self.navdata.bind( ('',NAVDATA_PORT) )
        if metalog.replay:
            self.commandSender = CommandSenderReplay(metalog.createLoggedSocket( "cmd", headerFormat="<BBBI" ), 
                    hostPortPair=(HOST, COMMAND_PORT), checkAsserts=metalog.areAssertsEnabled())
        else:
            self.commandSender = CommandSender(metalog.createLoggedSocket( "cmd", headerFormat="<BBBI" ), 
                    hostPortPair=(HOST, COMMAND_PORT))
        self.console = metalog.createLoggedInput( "console", myKbhit ).get
        self.metalog = metalog
        self.buf = ""
        self.battery = None
        self.flyingState = None
        self.flatTrimCompleted = False
        self.manualControl = False
        self.time = None
        self.altitude = None
        self.position = (0,0,0)
        self.speed = (0,0,0)
        self.positionGPS = None
        self.cameraTilt, self.cameraPan = 0,0
        self.lastImageResult = None
        self.navigateHomeState = None
        self.config()
        self.commandSender.start()
        
    def _discovery( self ):
        "start communication with the robot"
        filename = "tmp.bin" # TODO combination outDir + date/time
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # TCP
        s.connect( (HOST, DISCOVERY_PORT) )
        s.send( '{"controller_type":"computer", "controller_name":"katarina", "d2c_port":"43210"}' )
        f = open( filename, "wb" )
        while True:
            data = s.recv(10240)
            if len(data) > 0:
                f.write(data)
                f.flush()
                break
        f.close()
        s.close()

    def _update( self, cmd ):
        "internal send command and return navdata"
        
        #if not self.manualControl:
        #if self.manualControl:
        #    self.manualControl = self.console()
        #    if self.manualControl:
                # raise exception only once
        #        raise ManualControlException()

        # send even None, to sync in/out queues
        self.commandSender.send( cmd )

        while len(self.buf) == 0:
            data = self.navdata.recv(40960)
            self.buf += data
        data, self.buf = cutPacket( self.buf )
        return data

    def _parseData( self, data ):
        try:
            parseData( data, robot=self, verbose=False )
        except :
            print("AssertionError")


    def update( self, cmd=None, ackRequest=False ):
        "send command and return navdata"
        if cmd is None:
            data = self._update( None )
        else:
            data = self._update( packData(cmd, ackRequest=ackRequest) )
        while True:
            if ackRequired(data):
                self._parseData( data )
                data = self._update( createAckPacket(data) )
            elif pongRequired(data):
                self._parseData( data ) # update self.time
                data = self._update( createPongPacket(data) )
            
            else:
                break
        self._parseData( data )
        return data




    def config( self ):
        # initial cfg
        dt = self.metalog.now()
        if dt: # for compatibility with older log files
            self.update( cmd=setDateCmd( date=dt.date() ) )
            self.update( cmd=setTimeCmd( time=dt.time() ) )
        for cmd in setSpeedSettingsCmdList( maxVerticalSpeed=1.0, maxRotationSpeed=90.0, 
                hullProtection=True, outdoor=True ):
            self.update( cmd=cmd )
        self.update( cmd=requestAllStatesCmd() )
        self.update( cmd=requestAllSettingsCmd() )
        self.moveCamera( tilt=self.cameraTilt, pan=self.cameraPan )


    def takeoff( self ):
        for i in xrange(10):
            print( i)
            self.update( cmd=None )
        
        print( "Taking off ...")
        self.update( cmd=takeoffCmd() )
        prevState = None
        for i in xrange(100):
            print( i)
            self.update( cmd=None )
            if self.flyingState != 1 and prevState == 1:
                break
            prevState = self.flyingState
        print( "FLYING")
        
    def land( self ):
        print( "Landing ...")
        self.update( cmd=landCmd() )
        for i in xrange(100):
            print( i)
            self.update( cmd=None )
            if self.flyingState == 0: # landed
                break
        print( "LANDED")
        for i in xrange(30):
            print( i)
            self.update( cmd=None )
        

    def hover( self ):
        self.update( cmd=movePCMDCmd( active=True, roll=0, pitch=0, yaw=0, gaz=0 ) )

    def emergency( self ):
        self.update( cmd=emergencyCmd() )

    def trim( self ):
        print( "Trim:")
        self.flatTrimCompleted = False
        for i in xrange(10):
            print( i)
            self.update( cmd=None )
        
        self.update( cmd=trimCmd() )
        for i in xrange(10):
            print( i)
            self.update( cmd=None )
            if self.flatTrimCompleted:
                break
   
    def takePicture( self ):
        self.update( cmd=takePictureCmd() )

    def moveCamera( self, tilt, pan ):
        "Tilt/Pan camera consign for the drone (in degrees)"
        self.update( cmd=moveCameraCmd( tilt=tilt, pan=pan) )
        self.cameraTilt, self.cameraPan = tilt, pan # maybe move this to parse data, drone should confirm that

    def resetHome( self ):
        self.update( cmd=resetHomeCmd() )


    def wait( self, duration ):
        print( "Wait"+str (duration))
        assert self.time is not None
        startTime = self.time
        while self.time-startTime < duration:
            self.update()

    def flyToAltitude( self, altitude, timeout=3.0 ):
        print( "Fly to altitude"+str( altitude) +"from"+str( self.altitude))
        speed = 20 # 20%
        assert self.time is not None
        assert self.altitude is not None
        startTime = self.time
        if self.altitude > altitude:
            while self.altitude > altitude and self.time-startTime < timeout:
                self.update( movePCMDCmd( True, 0, 0, 0, -speed ) )
        else:
            while self.altitude < altitude and self.time-startTime < timeout:
                self.update( movePCMDCmd( True, 0, 0, 0, speed ) )
        self.update( movePCMDCmd( True, 0, 0, 0, 0 ) )


