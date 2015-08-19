#!/bin/bash
# *********************************************************
# rosrun_mavcontrol.sh - Launch the MAV control application
# *********************************************************
source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws
rosrun mav_class mav_control.py

