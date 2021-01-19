## Ground station

## Functions:
- Listen to UDP packets in NatNet form sent from the Cyberzoo tracking switch to ground station's Ethernet Adapter.
- Parse and calculate position, velocity and euler angles in NED.
- Send the calculated drone states to the remote drone which is on the same network as the ground station's WiFi card.

## Problems
For current problems: see issues. Major promblem hard-coded rigid body and GPS lock not set for when body leaves the zoo (frame ID still keeps increasing even if Rigid body is already outside). And axes conventions need to be fixed.
