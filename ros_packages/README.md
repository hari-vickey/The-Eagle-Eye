# The_Eagle_Eye
## ros_packages

This folder contains multiple ROS packages which are used as the essential blocks for `building` the central monitoring system. To build this package use the below mentioned commands.

```bash
# Build and Source the Package
cd ~/ros_workspace/
catkin build # Repeat build if failed
source devel/setup.bash
# If you have sublime text
subl ~/.bashrc
# else if you have gedit
gedit ~/.bashrc
# Add your workspace to the bashrc file
source ~/ros_workspace/devel/setup.bash
```

## 1. SwarmRobot

This is the ultimate ROS Package which will be acting as the central monitoring system for the problem statement. This package contains various files for controlling the bot in bot simulation and real time applications. Also, this package is capable of interfacing all other packages to do the job.

Communication flowchart will be added soon.

The commands used to test the files is mentioned below in separate terminals or terminator

```bash
# Launch Camera Node
roslaunch usb_cam usb_cam.launch
# Launch Socket Server for Bots
roslaunch swarmrobot esp.launch
# Run the Aruco Detect Marker
rosrun swarmrobot detect.py
# Run Client Node
rosrun swarmrobot client.py
# Run Bot'n' Node 'n' ~ 1<=n<=4
rosrun swarmrobot bot'n'.py
```

## 2. usb_cam

This package is used to interface the USB camera connected to the USB port of the laptop to ROS Image message at 1280*720p resolution at 30fps.

The camera needed to be configured appropriate to the arena and make sure to add the camera product id and vendor id in the udev.rules in your system as write mode.

Refer Tutorial here : https://msadowski.github.io/ros-web-tutorial-pt2-cameras/

## 3.  rosserial

This Open Source package is used mainly for ROS and ESP socket communication. The one way communication of the bot and pc takes place with the IP address mentioned in the bot. Make sure to connect the bot and pc with same Wireless credentials.

```bash
# Command to get the IP of desired Wifi Address.
hostname -I
```

## 4. Bot Description

This package contains all the definition of the bot that is being developed in the fusion 360 in xacro format which is in turn converted using fusion2urdf plugin. This package is mainly used for simulation purposes.