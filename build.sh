#!/bin/bash
# Usage: ./build.sh <system_name> <generate_solver>
PROJECT_ROOT="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $PROJECT_ROOT
clear

export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$PROJECT_ROOT/acados/lib"
export ACADOS_SOURCE_DIR="$PROJECT_ROOT/acados"

. /opt/ros/noetic/setup.sh

# Source the workspace if it exists
if [ -f $PROJECT_ROOT/devel/setup.sh ]; then
  . $PROJECT_ROOT/devel/setup.sh
fi

BUILD_TYPE=RelWithDebInfo # Release, Debug, RelWithDebInfo, MinSizeRel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DPYTHON_VERSION=3

catkin build mpc_planner_$1
