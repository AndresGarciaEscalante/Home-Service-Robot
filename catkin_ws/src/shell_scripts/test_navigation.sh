#!/bin/sh
xterm  -e  "roslaunch /home/andres/Home-Service-Robot/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/launch/turtlebot_world.launch" &
sleep 5
xterm  -e  " roslaunch /home/andres/Home-Service-Robot/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/launch/amcl_demo.launch" & 
sleep 5
xterm  -e  " roslaunch /home/andres/Home-Service-Robot/catkin_ws/src/turtlebot_interactions/turtlebot_rviz_launchers/launch/view_navigation.launch"