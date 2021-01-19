
## Functions:
todo


## COMPILING binary for drone:
i. Recheck ip address for natnet optitrack udp packets. Tell this address to the natnet ground station source file.
ii. check UART port: on rpi zero w, it is `/dev/ttyS0` on tx2, it is `/dev/ttyTHS2` on your computer it is `/dev/ttyUSB0`.

```
mkdir build
cd build
cmake ..
make
```
Cross compling is not setup, so compile on remote. You can scp this folder to the companion pc. And follow the above steps.
Please be careful with running this binary, do it only at Cyberzoo. ARM/DISARM/AI mode statemachine not tested enough.
run with: `./rpi-control`