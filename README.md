# Fake_GPS
This repository provides code to generate a fake GPS signal, aiding the drone's positioning in GPS-denied areas.

**Goal**: In a GPS-denied area, when the experimenter holding the drone moves, the GPS map in [Mission Planner](https://ardupilot.org/planner/) will immediately display their movement route.
![image](https://github.com/jiahaubai/Fake_GPS/blob/main/images/idea.png)


**Experiment Result**:
We conducted this experiment in the NTU MD Building, where GPS signals are not available. After using this code, we were able to successfully position the drone indoors.
[![Watch the video](https://github.com/jiahaubai/Fake_GPS/blob/main/images/video_cover.PNG)](https://drive.google.com/file/d/1eMuFeqP7EJF8AYZbDSvlK68_LksFRa6S/view?usp=sharing)

## Usage

### Hardware Requirements:
* Raspberry Pi 3B+ (RPi)        
* Pixhawk Fly Controller (FC) 4X on the drone
* USB A to USB C cable (connect FC to RPi)
* GIPS UWB sensors: 1 tag and 4 anchors
  
![image](https://github.com/jiahaubai/Fake_GPS/blob/main/images/hardware.png)



### Start Experiment:
* Step 1: Download this repository to your RPi. Please ensure that [MAVProxy](https://ardupilot.org/mavproxy/docs/getting_started/download_and_installation.html) is already installed on your RPi.  
* Step 2: Connect the flight controller (FC) and the tag to your RPi.  
* Step 3: Set up the anchors in the experimental area and record their coordinates (you can define them yourself). Additionally, please select an anchor as the latitude and longitude reference point and modify this information in the `main.py` file accordingly.
* Step 4: Open two terminals on your Raspberry Pi (RPi). One terminal is for the UDP Client, where main.py will be executed, and the other is for the UDP Server, where mavproxy.py will be executed. Please run mavproxy.py first, and make sure to turn on the drone remote controller simultaneously to ensure that mavproxy.py runs successfully.  
* Step 5: Open the Mission Planner and check whether your drone shows on the GPS map. If all the above execution goes smoothly, you can start taking the drone and move.

**Note**: In `Step 2` and `Step 3`, you need to modify the experiment settings you configured, such as the anchor positions and serial ports, in the main.py file starting from `line 97`.

```
####### anchor info                                    
    anchor_positions = [[0, 0, 1.1],
                        [0, 4.83, 1.19],
                        [27, 4.83, 1.1],
                        [27, 0, 1.11]]
                        
    ####### info of GPS_simulate func
    # The counterclockwise rotation angle between the y-axis (you define) and the north.
    rotate_theta = 322
    # The position of the reference anchor where its ENU coordinates is (0,0,1.1)
    lon_ref = 121.5444161
    lat_ref = 25.0176494
    height = 1.1
    # udp port
    port = 25100
```






