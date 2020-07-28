#!/bin/bash
source catkin_ws/devel/setup.bash
roslaunch car_interface all.launch veh:=default &>/dev/null &
