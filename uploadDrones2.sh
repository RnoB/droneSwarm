#!/bin/sh
sshpass -p raspberry scp -r ./Krang/swarmNet.py pi@192.168.0.152:~/RockSteady/
sshpass -p raspberry scp -r ./Krang/swarmNet.py pi@192.168.0.162:~/RockSteady/
sshpass -p raspberry scp -r ./RockSteady/* pi@192.168.0.152:~/RockSteady/
sshpass -p raspberry scp -r ./RockSteady/* pi@192.168.0.162:~/RockSteady/
#sshpass -p raspberry scp -r ./DroneSpecs/Drone_0002/* pi@192.168.30.12:~/RockSteady/