#!/usr/bin/env bash
set -euo pipefail

# 🛠 Run ROS Navigation Simulator

# 환경 설정
source devel/setup.bash
source fix_console.sh

# 실행
roslaunch mpc_planner_rosnavigation ros1_rosnavigation.launch
