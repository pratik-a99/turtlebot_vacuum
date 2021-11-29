# turtlebot_vacuum# turtlebot_vacuum
[![GitHub license](https://badgen.net/github/license/Naereen/Strapdown.js)](LICENSE.md)

This repository contains a ROS C++ program that modifies the behavior of turtlebot3 to behave like a roomba. A launch file is included, which can also record rosbag.

## Dependencies
* ROS (Melodic preferred)
* turtlebot3
* Gazebo

## Usage

### Downloading the repository

First clone the repository
```
https://github.com/pratik-a99/turtlebot_vacuum 
```

### Adding it to ROS workspace
Then, add the downloaded repository into your ROS workspace's src folder (eg. catkin_make/src)
```
cd ~/carkin_ws/src
```
To build the changes use
```
cd ..
catkin_make
```

#### Launch files

To use the launch files, use the following command in a terminal
```
roslaunch turtlebot_vacuum turtlebot3_house.launch 
```
#### Rosbag
To enable rosbag recording, the `record` argument can be set to `true` while using the above-mentioned launch files. `record` is set to `false` while in default, hence no recording will happen until it's set `true`. The recorded rosbags will be stored in the results folder. For example : 
```
roslaunch turtlebot_vacuum turtlebot3_house.launch record:=true
```


The recorded rosbag file can be inspected by using : 
```
rosbag info src/turtlebot_vacuum/result/recording.bag 
```

##### Replay
Use the following launch file to launch the robot and it's topics, without the walker behaviour.
```
roslaunch turtlebot_vacuum rosbag_replay.launch
```
and use the following command to replay the rosbag
```
rosbag play src/turtlebot_vacuum/result/recording.bag 
```
A short recorded bag has been added to the results folder, a longer recording can be accessed via this link:
[Rosbag Recording](https://drive.google.com/file/d/1Ke8aKBSZ-rgbdOsWO7zNIwCD4z7KKQwM/view?usp=sharing)

### Using Releases
If the the repository is downloaded as a zip via `Week13_HW_Release`, extract the file to `<your_ros_workspace>/src` (eg. `catkin_ws/src`) and rename the folder to `turtlebot_vacuum`
