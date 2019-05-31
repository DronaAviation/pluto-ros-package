# pluto-ros-package
This package canbe used to control Pluto or PlutoX using keyboard, joystick or rostopic 

## Getting Started 
Use following instructions on how to use this package:

###### Prerequisites

Follow this [tutorial](http://wiki.ros.org/joy) on ROS wiki and install necesarry package to use the joystick control feature of pluto-ros-package.

###### Installation

```
# Navigate to catkin workspace
roscd <WORKSPACE_NAME>

# clone repository
git clone https://github.com/DronaAviation/pluto-ros-package

# Build the Package
catkin_make 

```

###### Run Package

```
# pluto node for communication with pluto drone
rosrun plutodrone plutonode

# To get data from drone
rosrun plutoserver data_via_rosservice.py

# To control drone there are three ways: Joystick, Keyboard or ROSTopic 

# Using Joystick:
rosrun plutoserver plutojoystick

# Using Keyboard:
roslaunch plutoserver drone_comb.launch

# Using ROSTopic:
rostopic pub /drone_command plutodrone/PlutoMsg "{rcRoll: 1500, rcPitch: 1500, rcYaw: 1500, rcThrottle: 1000, rcAUX1: 0, rcAUX2: 0, rcAUX3: 0, rcAUX4: 1500}" // example of arming drone 

```
**Note: To control drone use one of the way at once otherwise you will endup over writing inputs from multiple nodes.**

###### Use Lewei Camera wifi of Pluto instead of ESP wifi then edit following lines in [Communication.cpp](/plutodrone/src/Communication.cpp)
```
addr.sin_port = htons(CAMERA_PORT);
addr.sin_addr.s_addr = inet_addr(CAMERA_IP_ADDRESS);
```
## Service

* plutoservice in [PlutoPilot](/plutodrone/srv) : This service request data from Pluto drone. This service gives the data like accelerometer, gyro, Magneto, altitude and battery value.

## Topic

* drone_command in [PlutoMsg](/plutodrone/msg): This topic can be used to give RC input to drone



## Joystick To Control Pluto

The package uses following axes and buttons index to control drone:

* Axes
  - 2 : To control ROLL
  - 3: To control Pitch
  - 0: To control Yaw
  - 1: To control Throttle
  - 4: To trim roll
  - 5: To trim pitch

* Buttons
  - 4: To arm or disarm drone
  - 3: To take off 
  - 1: To land
  - 5: To save trim values

**Note: You can change this mapping in [plutojoystick.h](/plutoserver/include/plutoserver/plutojoystick.h)**



## Keyboard To Control Pluto

Following are the keys to control mapping:

- spacebar : arm or disarm
- w : increase height
- s : decrease height
- q : take off
- e : land
- a : yaw left
- d : yaw right
- Up arrow : go forward
- Down arrow : go backward
- Left arrow : go left
- Right arrow : go right
- Ctrl-C: to quit



## ROSTopic To Control Pluto

###### Arm
```
rostopic pub /drone_command plutodrone/PlutoMsg "{rcRoll: 1500, rcPitch: 1500, rcYaw: 1500, rcThrottle: 1000, rcAUX1: 1500, rcAUX2: 1500, rcAUX3: 1500, rcAUX4: 1500}"
```
###### Disarm 
```
rostopic pub /drone_command plutodrone/PlutoMsg "{rcRoll: 1500, rcPitch: 1500, rcYaw: 1500, rcThrottle: 1500, rcAUX1: 1500, rcAUX2: 1500, rcAUX3: 1500, rcAUX4: 1000}"
```
###### Increase Roll value to move forward with respect to x-axis
```
rostopic pub /drone_command plutodrone/PlutoMsg "{rcRoll: 1600, rcPitch: 1500, rcYaw: 1500, rcThrottle: 1500, rcAUX1: 1500, rcAUX2: 1500, rcAUX3: 1500, rcAUX4: 1500}"
```
###### Decrease Roll value to move backward with respect to x-axis
```
rostopic pub /drone_command plutodrone/PlutoMsg "{rcRoll: 1400, rcPitch: 1500, rcYaw: 1500, rcThrottle: 1500, rcAUX1: 1500, rcAUX2: 1500, rcAUX3: 1500, rcAUX4: 1500}"
```
###### Increase Pitch value to move forward/left with respect to y-axis
```
rostopic pub /drone_command plutodrone/PlutoMsg "{rcRoll: 1500, rcPitch: 1600, rcYaw: 1500, rcThrottle: 1500, rcAUX1: 1500, rcAUX2: 1500, rcAUX3: 1500, rcAUX4: 1500}"
```
###### Decrease Pitch value to move backward/right with respect to y-axis
```
rostopic pub /drone_command plutodrone/PlutoMsg "{rcRoll: 1500, rcPitch: 1400, rcYaw: 1500, rcThrottle: 1500, rcAUX1: 1500, rcAUX2: 1500, rcAUX3: 1500, rcAUX4: 1500}"
```
###### Increase Throttle value to move up with respect to z-axis
```
rostopic pub /drone_command plutodrone/PlutoMsg "{rcRoll: 1500, rcPitch: 1500, rcYaw: 1500, rcThrottle: 1800, rcAUX1: 1500, rcAUX2: 1500, rcAUX3: 1500, rcAUX4: 1500}"
```
###### Decrease Throttle value to move down with respect to z-axis
```
rostopic pub /drone_command plutodrone/PlutoMsg "{rcRoll: 1500, rcPitch: 1500, rcYaw: 1500, rcThrottle: 1200, rcAUX1: 1500, rcAUX2: 1500, rcAUX3: 1500, rcAUX4: 1500}"
```
###### Increase Yaw value to rotate in clockwise direction
```
rostopic pub /drone_command plutodrone/PlutoMsg "{rcRoll: 1500, rcPitch: 1500, rcYaw: 1800, rcThrottle: 1500, rcAUX1: 1500, rcAUX2: 1500, rcAUX3: 1500, rcAUX4: 1500}"
```
###### Decrease Yaw value to rotate in anti-clockwise direction
```
rostopic pub /drone_command plutodrone/PlutoMsg "{rcRoll: 1500, rcPitch: 1500, rcYaw: 1200, rcThrottle: 1500, rcAUX1: 1500, rcAUX2: 1500, rcAUX3: 1500, rcAUX4: 1500}"
```
**Note: RC values ranges from 1000 to 2000**



## Multiple Drones

Following is the procedure to control multiple drones within the same network:

###### Setting the drone in client mode: Connect to drone wifi and use following command to open telnet connection: 
```
telnet 192.168.4.1 // drone wifi ip
```
###### Set the drone in both Station(STA) and Access Point Mode(AP) : Use following command:
```
+++AT MODE 3
```
###### Set the ssid and password: Use following command:
```
+++AT STA ssid password
```
###### Add IPs: Start your hotspot and your drone should connect to the hotspot. Note the IP address assigned to it. Edit following lines in [PlutoSwarm.cpp](/plutodrone/src/PlutoSwarm.cpp). Repeat this for all new drones which are added to the network. 
```
all_ips.push_back(&quot;192.168.43.151&quot;);
all_ips.push_back(&quot;&quot;);https://github.com/DronaAviation/pluto-ros-package/blob/master/plutodrone/src/PlutoSwarm.cpp
```
###### Send data: Follow procedure in ROSTopic Header to give commands to fly the drones. Add plutoIndex in PlutoMsg for every topic. This index is the same as the index of the IP within 'all_ips' when you add it. 

###### TODO - Get drone data from multiple drones

**Note: This feature is possible with ESP wifi of Pluto and will not work over Lewei camera wifi.**


## Contact

For any queries related to this package comment on repository or mail on developers@dronaaviation.com

## Acknowledgments

This package is improved version of https://github.com/simmubhangu/pluto_drone.git

Thanks to [Simranjeet Singh](https://github.com/simmubhangu) for developing original version of this package. 

