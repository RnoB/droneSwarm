# droneSwarm
Controller for a Swarm of Bicameral Visual Drone.

The bicameral drone is composed of

1 x Bebop 2 drone
2 x Camera Waveshare J (220 fisheye)
2 x Raspberry pi zero
1 x Powerbank Energizer AP-750


Rocksteady should be installed on each side of the brain. The left side sends command to the drone through direct wifi connection. The right side connects to the router and transmits commands to the left.  

An additional command for a central router is provided (Krang). this router is currently composed of

1 x raspberry pi 2
1 x sense hat

The purpose of this connection is to send take off and landing commands to each drone of the swarm with the switch on the sense hat. Additionally, the LED panel provides limited information on the state of the drone's brain. Potentially, if one does not care about fail-safe command, the router can be completely removed from the whole system. 
