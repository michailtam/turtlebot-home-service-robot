# Robotics Simulation - Home Service Robot - NOTE: THE README IS NOT READY NOW
In this robotics simulation a custom created *mobile robot* acts like an autonomous home service robot which navigates in a building and executes cleaning operations. Unlike, the cleaning robot the robot presented in the project simulates a pickup and delivery operation. This operation does not really be executed but instead it will be simulated by picking up a marker from one zone and delivering it to another one. Such robots are usually used in warehouses, where robots deliver products from one area to another and organize them in shelfs. Amazon is one of those companies that uses these kinds of robots. Nevertheless, navigating in an area full of obstacles like other shelves or robots is not an easy task. It is even more difficult for a robot if it does not have a map in it's disposal to know where each navigation point is. Also, unlike humans robots are not so smart to detect an obstacle and avoid colliding with it. So how does a robot bypass these problems? At first, the robot has to know where it is located. For this, it has to execute Localization which gets shown in the project [Where Am I](https://github.com/michailtam/where-am-i), where a particle filter gets used to locate the robot in the environment. Right localization is prerequisite, otherwise the robot will not be able to create a map which is the following step. To create this map the robot performs [*Simultaneous-Localization-And-Mapping - SLAM*](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) which is a combination of localization and mapping simultaneously. The accuracy of the map depends on the accuracy of the localization and mapping. An example of how SLAM works is shown in the project [Map My World](https://github.com/michailtam/robotics-map-my-world). All these algorithms sound complex because they obey mathematical concepts and need some time to understand. Fortunately, ROS provides us with special packages where these algorithms are fully implemented and do not need to be programmed from scratch. These packages will be listed in the installation section. 

## Development System
The simulation was developed and tested on the operating system Ubuntu 20.04 LTS using the simulator [gazebo](http://gazebosim.org/) and the ROS version [Noetic](https://www.ros.org/). To perform SLAM the [RTAB-Map package](http://wiki.ros.org/rtabmap_ros) was used which implements a RGB-D SLAM approach with real-time constraints. To observe the behavior visually the visualization tool [Rviz](http://wiki.ros.org/rviz) was used which displayed the resulted map in 3D.

## Features
- Localization
- SLAM
- Autonomous navigation
- Visual observation by Rviz
- Different tasks get executed (e.g. navigating to different zones)
- Odometry data get evaluated...


## Prerequisites
The user does not need to know anything about the details of the project, if he only wants to see how the robot dperforms the tasks. This is because there are test shell scripts (.sh) available which get executed after issuing them (described in the installation section). For a detailed understanding of the code is implemented and how the algorithms work, a solid understanding of C++, Python and ROS is necessary.   

The project was tested on Ubuntu 20.04 LTS using running gazebo and Rviz. and all the required packages are installed. The installation instructions can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) and [here](http://gazebosim.org/tutorials?tut=install_ubuntu).


## Recommended Books
- [ROS Robotics By Example](https://www.packtpub.com/product/ros-robotics-by-example/9781782175193)
- [Mastering ROS for Robotics Programming](https://www.amazon.com/Mastering-ROS-Robotics-Programming-Operating/dp/1788478959)
- [Probabilistic Robotics](https://mitpress.mit.edu/books/probabilistic-robotics) - (higly recommended)
- [Roboter mit ROS](https://dpunkt.de/produkt/roboter-mit-ros/) - (highly recommended)

## Installation
To install the project with it's necessary packages, please follow the steps bellow. If you encounter any problems please refer to the [discussion forum of ROS](https://discourse.ros.org/) to get further help.
Catkin workspace

### ROS Packages
RTAB-Map (Realtime-Appereance-Based-Mapping), AMCL, turtlebot_teleop, pgm_map_creator, gmapping

Map creation
```$ git clone https://github.com/udacity/pgm_map_creator.git```

### ROS Navigation Stack
```
$ sudo apt-get install ros-noetic-navigation
$ sudo apt-get install ros-noetic-map-server
$ sudo apt-get install ros-noetic-move-base
$ sudo apt-get install ros-noetic-amcl

or

git clone -b noetic-devel https://github.com/ros-planning/navigation.git
```

<rosparam file="$(find udacity_bot)/config/costmap_common_params.yaml" command="load" ns="global_costmap" />
<rosparam file="$(find udacity_bot)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
<rosparam file="$(find udacity_bot)/config/local_costmap_params.yaml" command="load" />
<rosparam file="$(find udacity_bot)/config/global_costmap_params.yaml" command="load" />
<rosparam file="$(find udacity_bot)/config/base_local_planner_params.yaml" command="load" />

### ROS Teleop Package
[Teleop Package](http://wiki.ros.org/turtlebot_teleop)
$ git clone https://github.com/ros-teleop/teleop_twist_keyboard.git

After building package
```
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### ROS Dependencies
rosdep check <package name>
rosdep install -i <package name>

ROS packages online https://www.ros.org/browse/list.php

cd /home/workspace/catkin_ws/ 
$ source devel/setup.bash
$ rosdep check simple_arm

### Clone the repository

1. Clone the repository ```$ git clone https://github.com/michailtam/map-my-world.git```
2. Change into the **src** folder ```$ cd src``` and initialize the workspace ```$ catkin_init_workspace```
3. Return to the toplevel catkin folder and build the packages```$ catkin_make```
4. You also need to install the following packages:
```
$ git clone -b noetic-devel https://github.com/ros-planning/navigation.git
```

## Running the simulation:
- test_slam.sh
- test_navigation.sh
- pick_objects.sh
- add_marker.sh
- home_service.sh

There are different .sh test shell scripts available which execute different task. For example, if you want to see how the markers get shown and hidden in a certain time stemp... issue ./add_markers.sh


To run the simulation follow the following steps:

Open a terminal, change into the toplevel of the catkin_workspace and issue: 
```./test_home_service_robot.sh```

Alternatively you can execute each test script seperately with the following steps:
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

## Rviz
Rviz is pre-configured with default parameters to execute SLAM. Feel free to adjust these parameters to your needs.

RViz stands for ROS Visualization. RViz is our one-stop tool to visualize all three core aspects of a robot: perception, decision-making, and actuation.
```
$ source devel/setup.bash
$ rosrun rviz rviz
```

Launch specific configuration file in rviz 
```$ rosrun rviz rviz -d /catkin_ws/src/<file_name.rviz>``` ???

While Gazebo is a physics simulator, RViz can visualize any type of sensor data being published over a ROS topic: camera images, point clouds, ultrasonic measurements, lidar data, inertial measurements, and more. This data can be a live stream directly from the sensor or pre-recorded data stored as a bagfile.

You can also visualize live joint angle values from a robot and hence construct a real-time 3D representation of any robot. Having said that, RViz is not a simulator and does not interface with a physics engine. So RViz models neither collisions nor dynamics. RViz is not an alternative to Gazebo, but rather a complementary tool to keep an eye on every single process under the hood of a robotic system.

### OPTIONAL: Camera
Topic /camera/rgb/image_raw

## rqt_graph
```$ rosrun rqt_graph rqt_graph```


## Screenshots
Show screenshot of rqt_graph

| **1st RTAB-DB Output** |
| :--- |
| Left: The created 2D map and the path the robot is traversing, Center: The output of the camera, Right: The constraints view | 
| **Screenshot** |
| <img src="https://github.com/michailtam/map-my-world/blob/master/images/rtabmap-dbviewer-start_0.png" alt="Map after SLAM" width="660" height="400" border="0" /> |

## Videos - formating
<a href="https://youtu.be/6FNHveEkFfM" target="_blank">
<img src="https://github.com/michailtam/map-my-world/blob/master/images/video_preview.png" alt="Map My World (ROS) Video" width="560" height="400" border="0" />

<a href="https://youtu.be/6FNHveEkFfM" target="_blank">
<img src="https://github.com/michailtam/map-my-world/blob/master/images/video_preview.png" alt="Map My World (ROS) Video" width="560" height="400" border="0" />
