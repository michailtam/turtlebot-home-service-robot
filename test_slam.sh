#!/bin/sh
xterm  -e  " source devel/setup.bash " &
sleep 5
xterm  -e  " roslaunch my_robot world.launch"
sleep 5
#xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch" & 
# sleep 5
# xterm  -e  " rosrun rviz rviz" 

