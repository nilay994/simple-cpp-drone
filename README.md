## readme log

1. porting msp was straightforward..
2. rpi required turning on ttyS0 port
3. no logic / hardware inversion etc required
4. betaflight (trashcan) required some work.. it doesn't have MSP override feature!! look at upgrading fw..
5. understood more about channel map and msp_override flag!! (written in notebook)
6. no way to do MSP and radio together on trashcan, currently only doing MSP
7. good that can kill from keyboard!!
8. added a ground wire for debugging PC / trashcan / rpi together
9. natnet works on rpi directly!! (no need to add wifi chip/ parse on remote)
10. usb on rpi seems to have a diode, but connector is too big.. 
11. well timed uart clocks only on the primary uart port.. (no second uart port available)
12. potential next step: high frequency IMU messages are possible in two ways:
	i. reuse MSP port that is used for motor commands (current one)
	ii. mavlink increased prio, refresh rate and baud after modifying betaflight fw (tested at 60Hz), raw imu + battery + ahrs estimates
13. rpi 4 is going to take atleast until mid-Feb, but can get the baseboard ready..
14. Freek's comments: bebop kernel and pprz threading was more help from parrot than us ourselves
15. udp and packet delivery should be fine, check endianness - latency and jitter will be profiled automatically when cyberzoo test happens
16. v4l2 buffers - no clear answer as to why cv.openvideostream was not chosen and extra mutex based vision buffers were used on bebop
17. actuators.c = msp_commands.c and imu/ahrs = vector_nav.c uart based
18. no tests with downward facing cameras, seems like holybro/bitcraze flowdeck have a good downward camera? tfmini-s on i2c is good enough for now.
19. rpi zero is known to have slow gpios - not sure where we see its effects
20. need button switches and leds for settings like: sensors turning on and off... esc turn off, betaflight turn off, rpi turn off, etc..
21. wifi antenna 0ohm resistor still needs to be diverted to listen to the external ufl connector.
22. mavlink `c_uart_example` worked, but not sure if I had to revert to original settings of the firmware.. remember turning on telemetry and seeing the IMU messages on console..
23. mavlink `c_uart_example` has v1 and v2 in the inner directory, which can be any version's (v1 or v2) git submodule in itself
24. never had trouble compiling c_uart_example mavlink test.
25. just that this mavlink based telemetry requires modifying the firmware and modifying the packet structure in betaflight.
