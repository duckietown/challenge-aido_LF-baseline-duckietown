#!/bin/bash
source /entrypoint.sh
source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash
dt-exec-BG roslaunch --wait duckietown_demos set_state.launch veh:="${VEHICLE_NAME}" state:="NORMAL_JOYSTICK_CONTROL"
rostopic pub /$VEHICLE_NAME/joy_mapper_node/car_cmd duckietown_msgs/Twist2DStamped '{header: {}, v: 0.0, omega: 0.0}' -1
