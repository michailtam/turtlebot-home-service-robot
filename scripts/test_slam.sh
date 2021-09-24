#!/bin/sh
xterm  -e  " source devel/setup.bash " &
sleep 5
xterm  -e  " roslaunch turtlebot3_gazebo world.launch" &
sleep 5
xterm  -e  " roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping" & 
sleep 5
xterm  -e  " roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch"
