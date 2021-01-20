1. `trashcan.dump`: default factory firmware (4.2.3) of trashcan. Dump before upgrading to 4.2.5
2. `kakute-default.diff`: factory kakute firmware, seems to have some nice DMA stuff!
3. `kakute-msp-override1.diff`: first try with F7 - bought one because mavlink pin conflicts on OMNIBUSF4 (wtf!) mavlink/custom uart serial rx didn't work yet! this diff could do MSP override for the first time, after studying: https://github.com/betaflight/betaflight/issues/8292 and https://github.com/nilay994/trytime/issues/2
4. `kakute-msp-override2.diff`: controller currently soldered on x220 after removing omnibusf4 from x220. Flight test on Jan15 at cyberzoo, MSP override worked for the first time. channel masks are difficult to figure out.
5. `trashcan-msp.diff`: current trashcan which can fly with RPi Zero W in the cyberzoo with MSP. MSP override is still in progress for this controller.
6. `x220-iros.diff`: Yingfu + Nishants diff file of the drone used at IROS 2019.



Cross compilation cmake could be something like this.. (Rpi zero = 32 bit and different than other Rpis in cross compilation)
```
SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_LIBRARY_ARCHITECTURE arm-linux-gnueabihf)

set(RASPBERRY_VERSION $ENV{RASPBERRY_VERSION})

# Specify the cross compiler
SET(CMAKE_C_COMPILER   /home/nilay/develop/rpi/tools/arm-bcm2708/arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc)
SET(CMAKE_CXX_COMPILER /home/nilay/develop/rpi/tools/arm-bcm2708/arm-linux-gnueabihf/bin/arm-linux-gnueabihf-g++)

# Where is the target environment
SET(CMAKE_FIND_ROOT_PATH /home/nilay/develop/rpi/rootfs)
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --sysroot=${CMAKE_FIND_ROOT_PATH}")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --sysroot=${CMAKE_FIND_ROOT_PATH}")
SET(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} --sysroot=${CMAKE_FIND_ROOT_PATH}")

# Search for programs only in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# Search for libraries and headers only in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
```
