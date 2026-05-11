#!/bin/bash

set -euo pipefail

source_root="${1:-/tmp/runtime-calibrations}"
target_root="${2:-/data/config/calibrations}"
vehicle_name="${3:-map_0/vehicle_0}"

install_default_and_vehicle() {
    local calibration_group="$1"
    local source_file="${source_root}/${calibration_group}/default.yaml"
    local vehicle_file="${target_root}/${calibration_group}/${vehicle_name}.yaml"

    if [[ ! -f "${source_file}" ]]; then
        echo "Missing calibration asset: ${source_file}" >&2
        exit 1
    fi

    mkdir -p "$(dirname "${vehicle_file}")"
    install -m 644 "${source_file}" "${target_root}/${calibration_group}/default.yaml"
    install -m 644 "${source_file}" "${vehicle_file}"
}

install_default_and_vehicle camera_intrinsic
install_default_and_vehicle camera_extrinsic
install_default_and_vehicle kinematics
