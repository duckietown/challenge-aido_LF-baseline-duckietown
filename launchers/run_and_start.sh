#!/bin/bash

source /environment.sh
echo "done environment.sh"
source /opt/ros/noetic/setup.bash
echo "done setup.bash"
source /code/catkin_ws/devel/setup.bash
echo "done catkin_ws setup.bash"
source /code/submission_ws/devel/setup.bash
echo "done submission setup.bash"
roscore &
echo "done roscore"
python3 solution.py &
echo "done solution.py"
roslaunch --wait car_interface all.launch veh:=$VEHICLE_NAME &
echo "done car car_interfacee roslaunch"
roslaunch --wait duckietown_demos lane_following.launch
echo "done lane_following roslaunch"
sleep 5
rostopic pub /$VEHICLE_NAME/fsm_node/mode duckietown_msgs/FSMState '{header: {}, state: "LANE_FOLLOWING"}'
echo "done rostopic pub"