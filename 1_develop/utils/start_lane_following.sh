#!/bin/bash
source ../catkin_ws/devel/setup.bash
rostopic pub /default/joy sensor_msgs/Joy -1 '
{axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
buttons: [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]}'