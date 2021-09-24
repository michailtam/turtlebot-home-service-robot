#!/bin/sh
xterm  -e  " source devel/setup.bash " &
sleep 5
xterm  -e  " roslaunch turtlebot3_gazebo world.launch" &
sleep 5
xterm  -e  " roslaunch my_robot amcl.launch" &
#sleep 5
#xterm  -e  " roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/map/map.yaml"
