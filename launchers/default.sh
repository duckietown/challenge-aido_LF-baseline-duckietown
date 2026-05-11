#!/bin/bash

set -euo pipefail

launcher_path="${DT_PROJECT_LAUNCHERS_PATH}/run_and_start.sh"
exec bash "${launcher_path}"
