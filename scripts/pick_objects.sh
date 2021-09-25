#!/bin/sh
xterm  -e  " source devel/setup.bash " &
sleep 5
xterm  -e  " roslaunch turtlebot3_gazebo world.launch" #&
sleep 5
xterm  -e  " roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/media/mikelap/DATA/GitRepos/turtlebot-home-service-robot/map/map.yaml" &
sleep 5
xterm  -e  " roslaunch turtlebot3_navigation amcl.launch" &
#sleep 5
#xterm  -e  " rosrun rviz rviz -d 'rospack find rvizConfig'/robot_model_nav.rviz" &
sleep 5
xterm  -e  " rosrun pick_objects pick_objects" 