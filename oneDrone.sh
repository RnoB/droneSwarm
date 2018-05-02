#!/bin/sh

sshpass -p raspberry ssh pi@192.168.0.153 "cd droneSwarm;git pull origin master \"$1\""
sshpass -p raspberry ssh pi@192.168.0.163 "cd droneSwarm;git pull origin master \"$1\""
sshpass -p raspberry ssh pi@192.168.0.153 "cd logs;sudo rm *.log"
sshpass -p raspberry ssh pi@192.168.0.163 "cd logs;sudo rm *.log"
sshpass -p raspberry ssh pi@192.168.0.153 "sudo telinit 6"
sshpass -p raspberry ssh pi@192.168.0.163 "sudo telinit 6"
