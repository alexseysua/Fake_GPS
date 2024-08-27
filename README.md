# Fake_GPS
This repository provides code to generate a fake GPS signal, aiding the drone's positioning in GPS-denied areas.

**Goal**: In a GPS-denied area, when the experimenter holding the drone moves, the GPS map in [Mission Planner](https://ardupilot.org/planner/) will immediately display their movement route.
![image](https://github.com/jiahaubai/Fake_GPS/blob/main/idea.png)


**Result**:
We conducted this experiment in the NTU MD Building. The below figure shows the results in the resting state, which indicates that we can successfully position the drone indoors. Here are also attached the results of the dynamic experiment. [(link)](https://drive.google.com/file/d/1eMuFeqP7EJF8AYZbDSvlK68_LksFRa6S/view?usp=drive_link)
![image](https://github.com/jiahaubai/Fake_GPS/blob/main/exp1.png)



## Usage

### Hardware Requirements:
* Raspberry Pi 3B+ (RPi)        
* Pixhawk fly controller 4X (FC) on the drone
* USB A to USB C cable (connect FC to RPi)
* GIPS UWB sensors: 1 tag and 4 anchors
  
![image](https://github.com/jiahaubai/Fake_GPS/blob/main/hardware.png)

### Start Experiment:
* Step 1: Download this repository to your RPi. Please make sure your RPi has already installed MAVProxy.  
* Step 2: Connect the FC and the tag to your RPi.  
* Step 3: Set up the anchors to the experimental area and record their coordinates (you can define by yourself).  
* Step 4: Open the two terminals from your RPi. One is for **UDP Client** where `main.py` is executed, and another is for **UDP Server** where `mavproxy.py` is executed. Please execute `mavproxy.py` first, also you have to open the drone remote controller at the same time to ensure that `mavproxy.py` can run successfully.  
* Step 5: Open the Mission Planner and check whether your drone shows on the GPS map. If all the above execution goes smoothly, you can start taking the drone and move.

**Note**: In `Step 2` and `Step 3`, you have to also modify their info, such as the anchor positions and serial ports, in the `main.py` starting from `line 97`.

```
####### anchor info                                    
    anchor_positions = [[0, 0, 1.1],
                        [0, 4.83, 1.19],
                        [27, 4.83, 1.1],
                        [27, 0, 1.11]]
                        
    ####### info of GPS_simulate func
    # rotation angle of y-axis
    rotate_theta = 322
    # the position of the reference anchor where its ENU coordinates is (0,0,1.1)
    lon_ref = 121.5444161
    lat_ref = 25.0176494
    height = 1.1
    # udp port
    port = 25100
```






