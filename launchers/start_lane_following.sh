#!/bin/bash

source /environment.sh
source /opt/ros/noetic/setup.bash

set -euo pipefail

source_if_present() {
    local setup_script="$1"

    if [[ -n "${setup_script}" && -f "${setup_script}" ]]; then
        source "${setup_script}" --extend
    fi
}

wait_for_fsm_ready() {
    local vehicle_name="$1"
    local mode_topic="/${vehicle_name}/fsm_node/mode"
    local set_state_service="/${vehicle_name}/fsm_node/set_state"

    until timeout 2 rostopic echo -n 1 "${mode_topic}" >/dev/null 2>&1; do
        echo "Waiting for ${mode_topic}..."
        sleep 1
    done

    until rosservice info "${set_state_service}" >/dev/null 2>&1; do
        echo "Waiting for ${set_state_service}..."
        sleep 1
    done
}

main() {
    local vehicle_name="${VEHICLE_NAME:-agent}"
    local set_state_service="/${vehicle_name}/fsm_node/set_state"

    source_if_present "${CATKIN_WS_DIR:-}/devel/setup.bash"
    source_if_present "/code/solution/devel/setup.bash"

    wait_for_fsm_ready "${vehicle_name}"

    until rosservice call "${set_state_service}" "state: 'LANE_FOLLOWING'" >/dev/null 2>&1; do
        echo "Retrying ${set_state_service}..."
        sleep 1
    done
}

main "$@"
