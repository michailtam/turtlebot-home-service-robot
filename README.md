# Turtlebot3 Robotics Simulation - Home Service Robot
In this robotics simulation a **Turtlebot3** robot acts like an autonomous home service robot which navigates in a building and executes cleaning operations. Unlike, the cleaning robot the robot presented in the project simulates a pickup and delivery operation. This operation does not really be executed but instead it will be simulated by picking up a marker from one zone and delivering it to another one. Such robots are usually used in warehouses, where robots deliver products from one area to another and organize them in shelfs. Amazon is one of those companies that uses these kinds of robots. Nevertheless, navigating in an area full of obstacles like other shelves or robots is not an easy task. It is even more difficult for a robot if it does not have a map in it's disposal to know where each navigation point is. Also, unlike humans robots are not so smart to detect an obstacle and avoid colliding with it. So how does a robot bypass these problems? At first, the robot has to know where it is located. For this, it has to execute Localization which gets shown in the project [Where Am I](https://github.com/michailtam/where-am-i), where a particle filter gets used to locate the robot in the environment. Right localization is prerequisite, otherwise the robot will not be able to create a map which is the following step. To create this map the robot performs [*Simultaneous-Localization-And-Mapping - SLAM*](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) which is a combination of localization and mapping simultaneously. The accuracy of the map depends on the accuracy of the localization and mapping. An example of how SLAM works is shown in the project [Map My World](https://github.com/michailtam/robotics-map-my-world). All these algorithms sound complex because they obey mathematical concepts and need some time to understand. Fortunately, ROS provides us with special packages where these algorithms are fully implemented and do not need to be programmed from scratch. These packages will be listed in the installation section. 

## Development System
The simulation was developed and tested on the operating system Ubuntu 20.04 LTS using the simulator [gazebo](http://gazebosim.org/) and the ROS version [Noetic](https://www.ros.org/). To perform SLAM the [RTAB-Map package](http://wiki.ros.org/rtabmap_ros) was used which implements a RGB-D SLAM approach with real-time constraints. To observe the behavior visually the visualization tool [Rviz](http://wiki.ros.org/rviz) was used which displayed the resulted map in 3D.

## Features
- Localization
- SLAM
- Autonomous navigation
- Object pickup and delivery task

## Recommended Books
- [ROS Robotics By Example](https://www.packtpub.com/product/ros-robotics-by-example/9781782175193)
- [Mastering ROS for Robotics Programming](https://www.amazon.com/Mastering-ROS-Robotics-Programming-Operating/dp/1788478959)
- [Probabilistic Robotics](https://mitpress.mit.edu/books/probabilistic-robotics) - (higly recommended)
- [Roboter mit ROS](https://dpunkt.de/produkt/roboter-mit-ros/) - (highly recommended)

## ROS Packages Used
The project requieres the following packages, to be able to perform Localization, Mapping and Navigation:

### ROS Localization
As described above the Localization process gets shown in the Where Am I project. The packages used for localization are the [ROS map_server](http://wiki.ros.org/map_server) to create the map, which the robot will use to locate it's location. The localization method which gets applied is the particle filter method. The package which implements this algorithm is the [ROS AMCL](http://wiki.ros.org/amcl) package. The package implements an adaptive version (AMCL) of the [Monte Carlo algorithm - MCL](https://en.wikipedia.org/wiki/Monte_Carlo_algorithm), which adjusts the number of particles over a period of time, as the robot navigates in a map. This offers a significant computational advantage in contrast to MCL.

ADD: At startup the robot is not localized properly, which means that the obstacles are not at the right place sensed. To fix this issue we need to place the robot at it's correct initial location. This gets done with the 2D Pose Estimate button by placing the robot in rviz so that it is aligned with right position and orientation in gazebo.

### ROS Mapping
As described above the Mapping process gets shown in the Map My World project. The package used for Mapping is the [RTAB-Map](http://wiki.ros.org/rtabmap_ros). The package implements a RGB-D SLAM approach with real-time constraints. The parameters of the algorithm can be adjusted using [this](http://wiki.ros.org/rtabmap_ros/Tutorials/Advanced%20Parameter%20Tuning) tutorial. A nice tool to observe loop-closures is the [Database viewer](https://github.com/introlab/rtabmap/wiki/Tools), which gets described [here](http://wiki.ros.org/rtabmap_ros). 

Mapping: A robot needs to have a map of its environment to be able to recognize where it has been moving around so far.

### ROS Navigation
The navigation of the robot gets applied by the [ROS Navigation Stack](http://wiki.ros.org/navigation). The package implements a 2D navigation stack that achieves information from odometry, sensor streams, and a goal pose and provides safe velocity commands that are sent to a mobile base. As you can see [here](https://github.com/ros-planning/navigation) the package implements a bunch of other packages like AMCL, Map Server etc. To navigate the robot by using the keyboard or a joystick the [Teleop](http://wiki.ros.org/turtlebot_teleop) package is needed.

Most of the above packages will already be downloaded with the repository, but in the case of that some are missing or do not work properly, please refer to the following [list](https://www.ros.org/browse/list.php) and search for the package. Also, if you encounter any problems please refer to the [discussion forum of ROS](https://discourse.ros.org/) to get further help. You can also check the dependencies of the packages by issuing:
```
$ rosdep check <package name>
$ rosdep install -i <package name>
```

The local planner (yellow) does nearly follow the global planner (black).

## Installation
To run the simulation, please follow the steps bellow. If you encounter any problems, please refer to the [discussion forum of ROS](https://discourse.ros.org/) to get further help.

Turtlebot3

1. $ sudo apt-get update

2. $ sudo apt-get upgrade

3. gedit ~/.bashrc

source /opt/ros/noetic/setup.bash
export TURTLEBOT3_MODEL=burger

4. 

1. Clone the repository
```
$ git clone https://github.com/michailtam/robotics-home-service-robot.git
```

5. Run the simulation: With the following shell scripts on one side different test operations can be performed (test_, pick_, add_), and on the other side the main task home service can be executed, which simulates the pickup and delivery task:
- [test_slam.sh] - Tests if SLAM is working
- [test_navigation.sh] - Tests if the robot can be navigated manually
- [pick_objects.sh] - Tests if the robot navigates autonomous to the pick up and drop off zone
- [add_marker.sh] - Tests if the markers are shown and hidden properly (marker shown at pick up zone -> marker hidden after 5 seconds at pickup zone -> marker shown after 5 seconds at the drop off zone)
- [home_service.sh] - This executes the pickup and delivery task. More precisely, the robot executes the following operations autonomously: Navigates to the pickup zone -> picks up the marker -> navigates to the drop zone and delivers the marker.

To execute one of the above shell scripts change in the toplevel of the catkin ws and issue:
```
$ source devel/setup.bash
$ ./<name_of_shell_script>.sh
```

Alternatively, you can execute each script seperately by following the below steps. The disadvantage of this method is that you need to open for each launch file a seperate terminal:
```
$ source devel/setup.bash
$ roslaunch <package>/launch/<launch_file>.launch
```

## Rviz
If you need to save your own rviz configurations, you can open them by issuing:
```
$ source devel/setup.bash
$ $ rosrun rviz rviz -d /catkin_ws/src/<folder_of_the_rviz_conf>/<file_name>.rviz
```

## ROS rqt_graph Output
The topics which are used to perform navigation, image fetching, AMCL, marker visualization and map usage are shon in (blue). ROS nodes which were created in C++ by the developer to execute adding and and pick up operations are shown in (green).  
```$ rosrun rqt_graph rqt_graph```

| **Output of rqt_graph** |
| :--- |
| **Screenshot** |
| <img src="https://github.com/michailtam/robotics-home-service-robot/blob/master/images/rgt_graph_makred.png" alt="Output of rqt_graph" width="660" height="400" border="0" /></a> |

## Costmap Parameters
To achieve the resulting simulation a bunch of parameters had to be adjusted accordingly. The parameters are included in the folder **config** in the following files:

**costmap_common_params.yaml**
```
map_type: costmap

obstacle_range: 1.0 #3.0
raytrace_range: 1.0 #3.0

transform_tolerance:  0.2

robot_radius: 0.1
inflation_radius: 0.5 #0.35
cost_scaling_factor: 15

observation_sources: laser_scan_sensor

laser_scan_sensor: {sensor_frame: hokuyo, data_type: LaserScan, topic: /scan, marking: true, clearing: true}
```

**local_costmap_params.yaml**
```
local_costmap:
    global_frame: odom
    robot_base_frame: robot_footprint
    update_frequency: 5.0
    publish_frequency: 5.0
    width: 2.5
    height: 2.5
    resolution: 0.05
    static_map: false
    rolling_window: true
```

**global_costmap_params.yaml**
```
global_costmap:
    global_frame: map
    robot_base_frame: robot_footprint
    update_frequency: 5.0
    publish_frequency: 5.0
    width: 20.0
    height: 20.0
    resolution: 0.05
    static_map: true
    rolling_window: false
```

**base_local_planner_params.yaml**
```
controller_frequency: 10.0

TrajectoryPlannerROS:
    max_vel_x: 0.5 
    min_vel_x: 0.01
    max_vel_theta: 1.57 #1.00

    min_in_place_vel_theta: 0.1 #0.314

    acc_lim_theta: 3.14
    acc_lim_x: 2.0
    acc_lim_y: 2.0

    sim_time: 1.0

    vx_samples: 5.0
    vtheta_samples: 10.0

    pdist_scale: 0.6
    gdist_scale: 0.8
    occdist_scale: 0.02

    holonomic_robot: true

    meter_scoring: true #false
```

## Videos
### Home Service Robot Operation
The video shows how the mobile robot performs autonomously the pickup and drop off task shown in birds eye view.
<a href="https://www.youtube.com/watch?v=Lg-u6IF4vuU&t=8s" target="_blank">
<img src="https://github.com/michailtam/robotics-home-service-robot/blob/master/images/home_service_op.png" alt="Home Service Operations" width="560" height="400" border="0" /></a>

### Rviz Output
The video shows how the robot navigates in the building autonomously while it senses the environment. You can see that obstacles are inflated (blue shape regions).
<a href="https://www.youtube.com/watch?v=p2wJKt1Fqu8" target="_blank">
<img src="https://github.com/michailtam/robotics-home-service-robot/blob/master/images/rviz_output.png" alt="Rviz output" width="560" height="400" border="0" /></a>

### 2D SLAM
The video shows how the robot navigates on the map and performs SLAM to create the 2d map.
<a href="https://www.youtube.com/watch?v=pGH0uAwmVvM&t=61s" target="_blank">
<img src="https://github.com/michailtam/robotics-home-service-robot/blob/master/images/2d_slam.png" alt="2D SLAM" width="560" height="400" border="0" /></a>