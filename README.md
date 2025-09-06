[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

# MPC Planner

This repository implements **navigation for logistics robots using Model Predictive Control (MPC)**.  
It provides both:
- **Simulation**: navigation of a virtual logistics robot, and  
- **Real experiments**: tested on a *Tracer Mini* robot in a real environment.

---

## 📦 Installation

Clone the repository:

```bash
git clone https://github.com/Kim-Ziho/mpc_planner_syscon.git
cd mpc_planner_syscon
```
Install system dependencies:
```bash
./install_requirements.sh
```

Run the setup script (installs Acados, sets up Python environment with pyenv, creates .venv, installs Python and ROS dependencies):
```bash
./setup.sh
```
## 🔨 Build

Build the workspace using the provided helper script:
```bash
# Example: build the rosnavigation system
./build.sh rosnavigation
```

This script configures Catkin with CMake flags and builds the selected package.

## ▶️ Run

To run the navigation in simulation:
```bash
./run_ros_navigation.sh
```

This will:

- Source the Catkin workspace and fix console encoding

- Launch the ROS navigation stack with `roslaunch mpc_planner_rosnavigation ros1_rosnavigation.launch`

## 📁 Scripts Overview

- **install_requirements.sh**

    Installs core system dependencies (pip, rosdep, colcon, catkin tools, etc.).

- **setup.sh**

    - Installs Acados at a pinned commit

    - Ensures `pyenv` and Python 3.8.10

    - Creates and configures a `.venv` environment

    - Installs Python dependencies (`requirements.txt`)

    - Resolves ROS dependencies via `rosdep`

- **build.sh**

    Helper script to build specific Catkin packages with the correct build type and environment.

- **run_ros_navigation.sh**

    Launches the ROS navigation simulator.

## 📝 License

This project is licensed under the Apache 2.0 License
