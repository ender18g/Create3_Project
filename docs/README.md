
# Create3 Project

A gazebo simulation environment and hardware libraries for executing SLAM and AprilTag detection with a Create3 robot, an RPLIDAR-A1 laser range scanner, and an Oak-D Lite camera. 

## About the Project
This project was created for EN.530.707 - Robot System Programming at Johns Hopkins University.

Contributors:
- Alex Alessi
- Allan Elsberry
- Spencer Powers
- Trevor Schwehr

## Project Media
![SLAM Robot](images/robot.gif)
![Slam Example](images/slam.gif)

## Setup this Repository
Clone this repository and build the packages:
```
git clone https://github.com/ender18g/Create3_Project.git
```

## Ignition Simulation Docs
After cloning this repository, execute the following commands from your workspace:

```
rosdep update --include-eol-distros
sudo apt-get update
rosdep install --from-path src -yi
colcon build
```



## Using the Hardware
### Starting Robot
Power on the create3 by holding down the power button. The Raspberry Pi 4 will boot and will automatically begin publishing Lidar Scan messages, camera images, along with robot information topics.

To drive the robot from a laptop, use teleop_twist_keyboard:
```
source /opt/ros/galactic/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Use the keyboard to command the robot

### SLAM
Open the SLAM toolbox:
```
ros2 launch create3_lidar slam_toolbox_launch.py
```

Start RVIZ2
```
ros2 run rviz2 rviz2 -d  ~/create3_examples_ws/install/create3_lidar/share/create3_lidar.rviz
```
