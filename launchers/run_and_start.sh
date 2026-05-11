#!/bin/bash

source /environment.sh
source /opt/ros/noetic/setup.bash

set -euxo pipefail

: "${LANE_CONTROLLER_K_THETA:=-2.5}"

source_if_present() {
    local setup_script="$1"

    if [[ -n "${setup_script}" && -f "${setup_script}" ]]; then
        source "${setup_script}" --extend
    fi
}

wait_for_ros_node() {
    local node_name="$1"

    until rosnode list 2>/dev/null | grep -Fxq "${node_name}"; do
        echo "Waiting for ${node_name}..."
        sleep 1
    done
}

wait_for_first_camera_frame() {
    local vehicle_name="$1"
    local image_topic="/${vehicle_name}/camera_node/image/compressed"
    local attempt

    for attempt in $(seq 1 20); do
        if timeout 2 rostopic echo -n 1 "${image_topic}" >/dev/null 2>&1; then
            return 0
        fi
        echo "Waiting for ${image_topic}..."
        sleep 1
    done

    echo "Proceeding without an image on ${image_topic}."
}

wait_for_lane_following_mode() {
    local vehicle_name="$1"
    local mode_topic="/${vehicle_name}/fsm_node/mode"
    local set_state_service="/${vehicle_name}/fsm_node/set_state"

    # The FSM advertises set_state before it finishes discovering all controlled
    # node switch services. Wait for the first latched mode message so the FSM
    # has completed initialization before forcing LANE_FOLLOWING.
    until timeout 2 rostopic echo -n 1 "${mode_topic}" >/dev/null 2>&1; do
        echo "Waiting for ${mode_topic}..."
        sleep 1
    done

    until rosservice info "${set_state_service}" >/dev/null 2>&1; do
        echo "Waiting for ${set_state_service}..."
        sleep 1
    done

    until rosservice call "${set_state_service}" "state: 'LANE_FOLLOWING'" >/dev/null 2>&1; do
        echo "Retrying ${set_state_service}..."
        sleep 1
    done
}

apply_lane_controller_speed_override() {
    local vehicle_name="$1"
    local lane_controller_v_bar="${LANE_CONTROLLER_V_BAR:-}"
    local v_bar_param="/${vehicle_name}/lane_controller_node/v_bar"

    if [[ -z "${lane_controller_v_bar}" ]]; then
        return 0
    fi

    until rosparam get "${v_bar_param}" >/dev/null 2>&1; do
        echo "Waiting for ${v_bar_param}..."
        sleep 1
    done

    rosparam set "${v_bar_param}" "${lane_controller_v_bar}"
    echo "Applied ${v_bar_param}=${lane_controller_v_bar}"
}

apply_lane_controller_param_override() {
    local vehicle_name="$1"
    local env_name="$2"
    local param_name="$3"
    local param_value="${!env_name:-}"
    local param_path="/${vehicle_name}/lane_controller_node/${param_name}"

    if [[ -z "${param_value}" ]]; then
        return 0
    fi

    until rosparam get "${param_path}" >/dev/null 2>&1; do
        echo "Waiting for ${param_path}..."
        sleep 1
    done

    rosparam set "${param_path}" "${param_value}"
    echo "Applied ${param_path}=${param_value}"
}

apply_lane_controller_param_overrides() {
    local vehicle_name="$1"

    apply_lane_controller_speed_override "${vehicle_name}"
    apply_lane_controller_param_override "${vehicle_name}" LANE_CONTROLLER_D_OFFSET d_offset
    apply_lane_controller_param_override "${vehicle_name}" LANE_CONTROLLER_K_D k_d
    apply_lane_controller_param_override "${vehicle_name}" LANE_CONTROLLER_K_THETA k_theta
    apply_lane_controller_param_override "${vehicle_name}" LANE_CONTROLLER_K_ID k_Id
    apply_lane_controller_param_override "${vehicle_name}" LANE_CONTROLLER_K_IPHI k_Iphi
    apply_lane_controller_param_override "${vehicle_name}" LANE_CONTROLLER_D_THRES d_thres
    apply_lane_controller_param_override "${vehicle_name}" LANE_CONTROLLER_THETA_THRES_MIN theta_thres_min
    apply_lane_controller_param_override "${vehicle_name}" LANE_CONTROLLER_THETA_THRES_MAX theta_thres_max
    apply_lane_controller_param_override "${vehicle_name}" LANE_CONTROLLER_OMEGA_FF omega_ff
}

main() {
    local vehicle_name="${VEHICLE_NAME:-agent}"
    local agent_pid=""

    source_if_present "${CATKIN_WS_DIR:-}/devel/setup.bash"
    source_if_present "/code/solution/devel/setup.bash"

    dt-exec-BG roscore
    dt-exec-BG roslaunch --wait agent agent_node.launch
    agent_pid="$!"

    wait_for_ros_node "/agent_node"

    dt-exec-BG roslaunch --wait car_interface default.launch veh:="${vehicle_name}"
    dt-exec-BG roslaunch --wait agent lane_following_headless.launch \
        veh:="${vehicle_name}"

    apply_lane_controller_param_overrides "${vehicle_name}"

    wait_for_first_camera_frame "${vehicle_name}"
    wait_for_lane_following_mode "${vehicle_name}"

    set +e
    wait "${agent_pid}"
    local status=$?
    set -e

    copy-ros-logs
    return "${status}"
}

main "$@"
