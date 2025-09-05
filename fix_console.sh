PROJECT_ROOT="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$PROJECT_ROOT/acados/lib"
export ACADOS_SOURCE_DIR="$PROJECT_ROOT/acados"
export DISABLE_ROS1_EOL_WARNINGS=1