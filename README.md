# LF Duckietown ROS Baseline

This repository is the ente ROS lane-following baseline for `aido-LF-sim-validation`.
It extends `duckietown/challenge-aido_lf-template-ros:ente-amd64` with the Duckietown lane-following launcher flow and runtime calibration defaults.

The baseline refreshes its Duckietown Python runtime dependencies from `dependencies.txt`, pulling the maintained repositories from GitHub instead of copying files from local sibling repos during the image build.

The repository now carries the same top-level file tree as `challenge-aido_LF-template-ros`, including a matching ROS package scaffold under `solution/`. The baseline image overlays that local `agent` package onto the template image and uses an ente-specific `agent/lane_following_headless.launch` wrapper together with the Duckiematrix bridge.
