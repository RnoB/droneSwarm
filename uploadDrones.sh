#!/bin/sh
sshpass -p raspberry scp -r ./Krang/swarmNet.py pi@192.168.0.151:~/RockSteady/
sshpass -p raspberry scp -r ./Krang/swarmNet.py pi@192.168.0.161:~/RockSteady/
sshpass -p raspberry scp -r ./RockSteady/* pi@192.168.0.161:~/RockSteady/
sshpass -p raspberry scp -r ./RockSteady/* pi@192.168.0.151:~/RockSteady/