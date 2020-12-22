## readme log

1. porting msp was straightforward.. 
2. rpi required turning on ttyS0 port, no logic / hardware inversion etc required
3. betaflight (trashcan) required some work.. it doesn't have MSP override feature!! look at upgrading fw..
4. understood more about channel map and msp_override flag!! (written in notebook)
5. no way to do MSP and radio together on trashcan, currently only doing MSP
6. good that can kill from keyboard!!
7. added a ground wire for debugging PC / trashcan / rpi together
8. natnet works on rpi directly!! (no need to add wifi chip/ parse on remote)
9. usb on rpi seems to have a diode, but connector is too big.. 
10. well timed uart clocks only on the primary uart port.. (no second uart port available at all, *making the choice of rpi not so smart anymore*), we need as many uarts as possible 1st port for MSP, 2nd port for MAVLink and a 3rd for GPS in future..
11. potential next step: high frequency IMU messages are possible in two ways:
    * reuse MSP port that is used for motor commands (current one)
    * mavlink increased prio, refresh rate and baud after modifying betaflight fw (tested at 60Hz), raw imu + battery + ahrs estimates
12. rpi 4 is going to take atleast until mid-Feb, but can get the baseboard ready..
13. Freek's comments: bebop kernel and pprz threading was more help from parrot than us ourselves
14. udp and packet delivery should be fine, check endianness - latency and jitter will be profiled automatically when cyberzoo test happens
15. v4l2 buffers - no clear answer as to why cv.openvideostream was not chosen and extra mutex based vision buffers were used on bebop
16. actuators.c = msp_commands.c and imu/ahrs = vector_nav.c uart based
17. no tests with downward facing cameras, seems like holybro/bitcraze flowdeck have a good downward camera? tfmini-s on i2c is good enough for now.
18. rpi zero is known to have slow gpios - not sure where we see its effects
19. need button switches and leds for settings like: sensors turning on and off... esc turn off, betaflight turn off, rpi turn off, etc..
20. wifi antenna 0ohm resistor still needs to be diverted to listen to the external ufl connector.
21. mavlink `c_uart_example` worked, but not sure if I had to revert to original settings of the firmware.. remember turning on telemetry and seeing the IMU messages on console..
22. mavlink `c_uart_example` has v1 and v2 in the inner directory, which can be any version's (v1 or v2) git submodule in itself
23. never had trouble compiling c_uart_example mavlink test.
24. just that this mavlink based telemetry requires modifying the firmware and modifying the packet structure in betaflight.
25. still having trouble with threading, multiple segfaults -> accessing members of an externed object pointer before it is instantiated
26. natnet.h required extern "C"! the undefined ref was a difficult catch, thought that CMakeLists was a problem, but it turned out to be .h vs extern in .h
27. add_library in CMakeLists was nice to know about!
28. vscode (bottom toolbar) can debug cpp code directly with breakpoints and gives the call stack and backtrace on segfaults -> might have to install the cmake support.
29. binary size is already 256KB! see how you can reduce (multiple serial.cpp) are in use atm.
30. The main problem is realiable threading and starting and killing the program and turning off motors on the drone.
31. hopefully forcing to call the destructors (which call fflush) on finish will not leave non terminated .csv files!

