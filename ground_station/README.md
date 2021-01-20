## Ground station

## Functions
- Listen to UDP packets in NatNet form sent from the Cyberzoo tracking switch to ground station's Ethernet Adapter.
- Parse and calculate position, velocity and euler angles in NED.
- Send the calculated drone states to the remote drone which is on the same network as the ground station's WiFi card.

## Compiling and running the ground station:
i. Make sure ethernet cable of cyberzoo is plugged into your computer and you have an IP from the switch (ifconfig on your computer)
ii. Make sure you are rigid body1. Currently hard coded to have one rigid body.
iii. Check the connected drone's IP address should be in the same network as your laptop's wifi adapter.
```
mkdir build
cd build
cmake ..
make
```
run the binary in the build folder with `./natnet-fwd`, it should start sending packets to your drone (if the remote IP is correct).

## Problems
For current problems: see issues. Major promblem hard-coded rigid body and GPS lock not set for when body leaves the zoo (frame ID still keeps increasing even if Rigid body is already outside). And axes conventions need to be fixed.
