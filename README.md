# Robotics Simulation - Home Service Robot - NOTE: THE README IS NOT READY NOW
In this robotics simulation a custom created *mobile robot* acts like an autonomous home service robot which navigates in a building and executes cleaning operations. Unlike, the cleaning robot the robot presented in the project simulates a pickup and delivery operation. This operation does not really be executed but instead it will be simulated by picking up a marker from one zone and delivering it to another one. Such robots are usually used in warehouses, where robots deliver products from one area to another and organize them in shelfs. Amazon is one of those companies that uses these kinds of robots. Nevertheless, navigating in an area full of obstacles like other shelves or robots is not an easy task. It is even more difficult for a robot if it does not have a map in it's disposal to know where each navigation point is. Also, unlike humans robots are not so smart to detect an obstacle and avoid colliding with it. So how does a robot bypass these problems? At first, the robot has to know where it is located. For this, it has to execute Localization which gets shown in the project [Where Am I](https://github.com/michailtam/where-am-i), where a particle filter gets used to locate the robot in the environment. Right localization is prerequisite, otherwise the robot will not be able to create a map which is the following step. To create this map the robot performs [*Simultaneous-Localization-And-Mapping - SLAM*](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) which is a combination of localization and mapping simultaneously. The accuracy of the map depends on the accuracy of the localization and mapping. An example of how SLAM works is shown in the project [Map My World](https://github.com/michailtam/robotics-map-my-world). All these algorithms sound complex because they obey mathematical concepts and need some time to understand. Fortunately, ROS provides us with special packages where these algorithms are fully implemented and do not need to be programmed from scratch. These packages will be listed in the installation section. 

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
The project requieres the following packages:

### Localization
As described above the Localization process gets shown in the Where Am I project. The packages used for localization are the [ROS map_server](http://wiki.ros.org/map_server) to create the map, which the robot will use to locate it's location. The localization method which gets applied is the particle filter method. The package which implements this algorithm is the [ROS AMCL](http://wiki.ros.org/amcl) package. The package implements an adaptive version (AMCL) of the [Monte Carlo algorithm - MCL](https://en.wikipedia.org/wiki/Monte_Carlo_algorithm), which adjusts the number of particles over a period of time, as the robot navigates in a map. This offers a significant computational advantage in contrast to MCL.

### Mapping
As described above the Mapping process gets shown in the Map My World project. The package used for Mapping is the [RTAB-Map](http://wiki.ros.org/rtabmap_ros). The package implements a RGB-D SLAM approach with real-time constraints. The parameters of the algorithm can be adjusted using [this](http://wiki.ros.org/rtabmap_ros/Tutorials/Advanced%20Parameter%20Tuning) tutorial. A nice tool to observe loop-closures is the [Database viewer](https://github.com/introlab/rtabmap/wiki/Tools), which gets described [here](http://wiki.ros.org/rtabmap_ros). 

### Navigation
The navigation of the robot gets applied by the [ROS Navigation Stack](http://wiki.ros.org/navigation). The package implements a 2D navigation stack that achieves information from odometry, sensor streams, and a goal pose and provides safe velocity commands that are sent to a mobile base. As you can see [here](https://github.com/ros-planning/navigation) the package implements a bunch of other packages like AMCL, Map Server etc. To navigate the robot by using the keyboard or a joystick the [Teleop](http://wiki.ros.org/turtlebot_teleop) package is needed.

Most of the above packages will already be downloaded with the repository, but in the case of that some are missing or do not work properly, please refer to the following [list](https://www.ros.org/browse/list.php) and search for the package. Also, if you encounter any problems please refer to the [discussion forum of ROS](https://discourse.ros.org/) to get further help. You can also check the dependencies of the packages by issuing:
```
$ rosdep check <package name>
$ rosdep install -i <package name>
```

## Installation
To run the simulation, please follow the steps bellow. If you encounter any problems, please refer to the [discussion forum of ROS](https://discourse.ros.org/) to get further help.

1. Clone the repository
```
$ git clone https://github.com/michailtam/robotics-home-service-robot.git
```

2. Build the project
After all packages and dependencies are installed properly, the project can be build by changing in the toplevel folder and issuing:
```
$ catkin_make
```

3. Source the workspace
```
$ source devel/setup.bash
```

4. Run the simulation
With the following shell scripts on one side different test operations can be performed (test_, pick_, add_), and on the other side the main task home service can be executed: 

- [test_slam.sh] - Tests if SLAM is working
- [test_navigation.sh] - Tests if the robot can be navigated manually
- [pick_objects.sh] - Tests if the robot navigates autonomous to the pick up and drop off zone
- [add_marker.sh] - Tests if the markers are shown and hidden properly (marker shown at pick up zone -> marker hidden after 5 seconds at pickup zone -> marker shown after 5 seconds at the drop off zone)
- [home_service.sh] - This executes the pickup and delivery task. More precisely, the robot executes the following operations autonomously: Navigates to the pickup zone -> picks up the marker -> navigates to the drop zone and delivers the marker.




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


## rqt_graph
```$ rosrun rqt_graph rqt_graph```

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
