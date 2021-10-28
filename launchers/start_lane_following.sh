#!/bin/bash
source /entrypoint.sh
source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash

dt-exec-FG roslaunch --wait duckietown_demos set_state.launch veh:="${VEHICLE_NAME}" state:="LANE_FOLLOWING"
