#!/bin/bash

source /environment.sh
source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash --extend
source /code/submission_ws/devel/setup.bash --extend
roscore &
roslaunch --wait agent agent_node.launch &
roslaunch --wait car_interface all.launch veh:=$VEHICLE_NAME &
roslaunch --wait duckietown_demos lane_following.launch &
sleep 5
# we put a short sleep in here because rostopic will fail if there's no roscore yet
rostopic pub /$VEHICLE_NAME/fsm_node/mode duckietown_msgs/FSMState '{header: {}, state: "LANE_FOLLOWING"}'
