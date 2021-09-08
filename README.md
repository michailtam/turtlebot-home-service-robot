# Robotics Simulation - Home Service Robot - NOTE: THE README IS NOT READY NOW
In this robotics simulation a custom *mobile robot* moves in a building like an [iRobot Vacuum](https://www.irobot.com/roomba/600-seriess) while traversing to random waypoints. The project combines multiple algorithms which are commonly used in robotics. These include, among others [*Simultaneous-Localization-And-Mapping - SLAM*](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) to map its environment where it is acting and localization to localize its position and orientation in the building with respect to the global frame. Feel free to visit the projects [where am i](https://github.com/michailtam/where-am-i) and [map my world](https://github.com/michailtam/robotics-map-my-world) to see how both algorithms work. The simulation gets executed in [gazebo](http://gazebosim.org/) using [ROS-Noetic](https://www.ros.org/).

### Features
- Combines mapping and localization - SLAM
- Traverses to random waypoints
- Executes path planning

### Prerequisites
This project assumes that you are using Ubuntu (tested on Ubuntu 20.4 LST) and that ROS (tested on noetic), gazebo and all the required packages are installed. The installation instructions can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) and [here](http://gazebosim.org/tutorials?tut=install_ubuntu).

### Installation
To install the repository and packages, please follow the bellow steps. If you encounter any problems please refer to the [discussion forum of ROS](https://discourse.ros.org/) to get further help.

1. Clone the repository ```$ git clone https://github.com/michailtam/map-my-world.git```
2. Change into the **src** folder ```$ cd src``` and initialize the workspace ```$ catkin_init_workspace```
3. Return to the toplevel catkin folder and build the packages```$ catkin_make```
4. You also need to install the following packages:
```
$ git clone -b noetic-devel https://github.com/ros-planning/navigation.git
```

### Running the simulation:
To run the simulation follow the following steps:

Open a terminal, change into the toplevel of the catkin_workspace and issue: 
```./test_home_service_robot.sh```

Alternatively you can execute each script seperately with the following steps:
1. Open a terminal change into the toplevel of the catkin workspace and issue
```
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
2. Open a second terminal (also change to toplevel) and issue
```
$ source devel/setup.bash
$ roslaunch my_robot amcl.launch
```

### Rviz
Rviz is pre-configured with default parameters to execute SLAM. Feel free to adjust these parameters to your needs.

#### Screenshots
| **1st RTAB-DB Output** |
| :--- |
| Left: The created 2D map and the path the robot is traversing, Center: The output of the camera, Right: The constraints view | 
| **Screenshot** |
| <img src="https://github.com/michailtam/map-my-world/blob/master/images/rtabmap-dbviewer-start_0.png" alt="Map after SLAM" width="660" height="400" border="0" /> |

#### Video
<a href="https://youtu.be/6FNHveEkFfM" target="_blank">
<img src="https://github.com/michailtam/map-my-world/blob/master/images/video_preview.png" alt="Map My World (ROS) Video" width="560" height="400" border="0" />