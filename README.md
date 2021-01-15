## README

## Definitions:
Companion: Your own computer (SiL), RPi Zero W, tx2, odroid or any similar remote computer mounted on your drone. (LittleEndian-ness check)
Computer: Your computer
Autopilot: Betaflight

### status 
![status](https://drive.google.com/uc?export=view&id=19W_tH0GL1MzeCuHXVomWUDyCzVb7xr2Z)
[edit here](https://drive.google.com/drive/folders/1s6eHkhIduhhTMLy5Cp-5nNKC-lqeFxvV?usp=sharing)

## folder structure:
- Similar to ROS: `include` hold the headers and `src` holds the source files.
- `CMakeLists.txt` holds the building, linking, debug/release, optimization options.
- `build` folder must be empty, `cmake..` is done here; binaries are run here. 
-`analyse`: this folder holds some anaylsis scripts on matlab that help determine the quality of current repository. Metrics like latency and jitter, helps know how good threading is, Rpi CM4 is better than Pi zero, jitter of 4ms+ is not okay for VIO.. etc etc
- `ground_station`: can be ignored for companion pc, must run on laptop with optitrack ethernet plugged in.

## Compiling
1. Ground station for optitrack:
i. Make sure ethernet cable of cyberzoo is plugged into your computer and you have an IP from the switch (ifconfig on your computer)
ii. Make sure you are rigid body1. Currently hard coded to have one rigid body.
iii. Check the connected drone's IP address should be in the same network as your laptop's wifi adapter.
```
cd ./ground_station 
mkdir build
cd build
cmake ..
make
```
run the binary in the build folder with `./natnet-fwd`, it should start sending packets to your drone.


2. binary for drone:
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

## contrib:
Please open issues without hesitation, issues like "this is uncomfortable/this is stupid" are craved for.
Ask me for push access if you don't have it. Please don't force push on master! Do make your branch if you'd like to contrib. Opening a PR will earn you coffee from the nicer machine in the Aerospace lobby.
