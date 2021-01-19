## README

## Definitions:
Remote: Your own computer (SiL), RPi Zero W, tx2, odroid or any similar remote computer mounted on your drone. (LittleEndian-ness check)

Computer: Your computer

Autopilot: Betaflight

### Status 
![status](https://drive.google.com/uc?export=view&id=19W_tH0GL1MzeCuHXVomWUDyCzVb7xr2Z)
[edit here](https://drive.google.com/drive/folders/1s6eHkhIduhhTMLy5Cp-5nNKC-lqeFxvV?usp=sharing)

## Folder structure:
- Similar to ROS: `include` hold the headers and `src` holds the source files.
- `CMakeLists.txt` holds the building, linking, debug/release, optimization options.
- `build` folder must be empty, `cmake..` is done here; binaries are run here. 
-`analyse`: this folder holds some anaylsis scripts on matlab that help determine the quality of current repository. Metrics like latency and jitter, helps know how good threading is, Rpi CM4 is better than Pi zero, jitter of 4ms+ is not okay for VIO.. etc etc
- `ground_station`: can be ignored for companion pc, must run on laptop with optitrack ethernet plugged in.
- `remote`: the folder that must be "scp"-ed into the remote drone and this folder must build and run there.


## Contrib:
Please open issues without hesitation, issues like "this is uncomfortable/this is stupid" are craved for.
Ask me for push access if you don't have it. Please don't force push on master! Do make your branch if you'd like to contrib. Opening a PR will earn you coffee from the nicer machine in the Aerospace lobby.
