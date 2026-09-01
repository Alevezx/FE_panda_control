# Instructions to get ready with collision avoidance demos

There are two possible solutions:
* [**Virtual skeleton**](#Virtual-skeleton-collision-avoidance-demo)
* [**Real-time skeleton**](#Real-time-skeleton-collision-avoidance-demo)

## Setup
* **Xavier PC** with yellow label (password written on it)
* **This PC** with pink label (password written on it)
* **Realsense camera with tripods**
* **Panda Robot**
* **Aruco marker**

## Initialize the robot
Switch on the robot controller manually.\
Set Ethernet(enp4s0) on the top-right corner of the screen to "Right robot".\
On Chrome, search the Desk page by digiting ip "172.16.0.2" in the URL search bar.\
If the page would be recognized as unsecure, click option on advanced options ad proceed the same.\
Click the "Unlock" icon under the section "Joints" in the right side of the page.\
If the robot light is blue, robot is ready to work.\
If the robot light is white, robot is in free-ride mode, and you got to unlock the change mode button on the desk.\
To move robot in free-ride mode you got to grab the end effector and press at the same time the two buttons placed on it.\
Then you can move the robot as yuo want by pulling and pushing it.\

## Marker calibration
You don't have to perform the marker calibration in case you haven't moved it.\
Place the calibration tool (plastic cone with round base, grey or black) in the end effector with the screws.\
Press the change mode button on the desk to enable free-ride mode (robot light should be white).\
To calibrate, write on terminal:
```shell script
roslaunch collision_avoidance marker.launch 
```
Then follow the instructions.

## Camera calibration
Connect the camera to an Usb 3.0 port (blue ones) of this PC.\
To check that the marker is inside the range of view of the camera,write on terminal:
```shell script
realsense-viewer
```
Switch on the streaming turning on the RGB Camera icon.\
Adjust manually the camera until the marker is displayed and than you can close the program.\
To calibrate, write on terminal:
```shell script
roslaunch collision_avoidance calibrator.launch 
```

# Virtual skeleton collision avoidance demo

## Stop Event Demo
To start the demo, write on terminal:
```shell script
roslaunch collision_avoidance demo.launch virt:=true
```
Then follow the instructions.

## Impedance Control Demo
To start the demo, write on terminal:
```shell script
cd ~/franka_emika_ws/src/libfranka/build/examples/
./demo_impedance_control_traj 172.16.0.2 virt
```
Then follow the instructions.

## Admittance Control Demo
To start the demo, write on terminal:
```shell script
cd ~/franka_emika_ws/src/libfranka/build/examples/
./demo_admittance_control_traj 172.16.0.2 virt
```
Then follow the instructions.


# Real-time skeleton collision avoidance demo

## Connection between PCs
Switch on the Xavier PC and connect the camera to the Usb 3.0 port placed near the ethernet port.\
Connect also keyboard and mouse if not already connected to Xavier PC.\
Connect one side of the ethernet cable to the port on the Xavier PC and the other side to the lower port on this PC.\
On Xavier PC, set cabled connection on the top-right corner of the screen to "UDP connection".\
On this PC, set Ethernet(enp3s0) on the top-right corner of the screen to "UDP connection".\
On this PC, to check the connection, write on terminal:
```shell script
ping 172.16.10.1
```

## Camera streaming
To start the streaming, on Xavier PC, write on terminal:
```shell script
cd ~/Libraries/openpose
./build/examples/user_code/skeleton_tracker.bin
```
If you want to display the streaming, write that on terminal instead:
```shell script
./build/examples/user_code/skeleton_tracker.bin disp 
```
Make sure the program is correctly running (there could be a segmentation fault error if there's no one displayed on the screen).

## Stop Event Demo
To start the data acquisition from camera, on this PC, write on terminal:
```shell script
cd ~/franka_emika_ws/src/libfranka/build/examples/
./coord_handler
```
To start the demo, on this PC, write on another terminal:
```shell script
roslaunch collision_avoidance demo.launch
```
Then follow the instructions.

## Impedance Control Demo
To start the data acquisition from camera, on this PC, write on terminal:
```shell script
cd ~/franka_emika_ws/src/libfranka/build/examples/
./coord_handler
```
To start the demo, on this PC, write on another terminal:
```shell script
cd ~/franka_emika_ws/src/libfranka/build/examples/
./demo_impedance_control_traj 172.16.0.2
```
Then follow the instructions.

## Admittance Control Demo
To start the data acquisition from camera, on this PC, write on terminal:
```shell script
cd ~/franka_emika_ws/src/libfranka/build/examples/
./coord_handler
```
To start the demo, on this PC, write on another terminal:
```shell script
cd ~/franka_emika_ws/src/libfranka/build/examples/
./demo_admittance_control_traj 172.16.0.2
```
Then follow the instructions.
