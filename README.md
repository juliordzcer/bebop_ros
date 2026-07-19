# bebop_ros

ROS 2 packages to simulate a swarm of Parrot Bebop 2 drones in Gazebo (gz-sim): bearing-based formation control, a PID trajectory controller, demo launch files, a Qt GUI and the Gazebo assets/plugins that make it fly.

Tested with **ROS 2 Jazzy** + **Gazebo Harmonic** on Ubuntu 24.04.

## Quick start

```bash
mkdir -p ~/ws_bebop/src
cd ~/ws_bebop/src
git clone https://github.com/juliordzcer/bebop_ros.git

# Detects missing dependencies; add --install to install them automatically.
./bebop_ros/scripts/check_dependencies.sh --install

cd ~/ws_bebop
colcon build
source install/setup.bash

ros2 launch bebop_demo bebop1.launch.py
```

A single `colcon build` at the workspace root builds every package, including the Gazebo (gz-sim) plugins in `bebop_gz`. `source install/setup.bash` sets `GZ_SIM_RESOURCE_PATH`, `GZ_SIM_SYSTEM_PLUGIN_PATH` and `GZ_VERSION` automatically — no manual `.bashrc` editing needed, and the workspace can live anywhere (it doesn't have to be `~/ws_bebop`).

## Requirements

### 1. ROS 2 (skip if already installed)

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install -y ros-dev-tools ros-jazzy-desktop
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && source ~/.bashrc

sudo rosdep init   # skip if already initialized
rosdep update
```

### 2. Gazebo Harmonic (skip if already installed)

```bash
sudo apt-get update && sudo apt-get install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update && sudo apt-get install gz-harmonic
```

### 3. Workspace dependencies

After cloning (see Quick start), `scripts/check_dependencies.sh` checks:
- ROS package dependencies from each `package.xml` (via `rosdep`)
- Gazebo Harmonic dev libraries (`libgz-*-dev`) — not reliably resolvable via `rosdep` on all distros, so checked directly
- Python packages the nodes import (`numpy`, `scipy`, `matplotlib`, `transforms3d`)

```bash
./bebop_ros/scripts/check_dependencies.sh           # report only
./bebop_ros/scripts/check_dependencies.sh --install # report and install (sudo will prompt)
```

Do **not** run it as `sudo ./check_dependencies.sh ...` — it calls `sudo` itself for the one step that needs it (`apt-get`). Running the whole script as root breaks `rosdep` (its cache lives under your user's `$HOME`) and makes `pip` install into system directories instead of your user's, which can leave two conflicting copies of the same package installed.

## Running the demos

```bash
# Bearing-based formation control, 4 agents
ros2 launch bebop_bearings bebop.launch.py

# Single-agent PID trajectory tracking (setpoint + GUI)
ros2 launch bebop_demo bebop1.launch.py

# Single-agent PID trajectory tracking (alternate demo)
ros2 launch bebop_demo bebop_tracking.launch.py

# Swarm formation (pyramid / straight-line)
ros2 launch bebop_demo swarmbebop.launch.py

# Qt GUI on its own
ros2 launch bebop_gui bebop_gui.launch.py
```

## Package Overview

### bebop_bearings
Multi-agent bearing-based formation control for 4 agents.

### bebop_controller
Position-based PID controller for single-agent trajectory tracking.

### bebop_demo
Utility and demo nodes: `set_pose` (initial drone positions), `setpoint` (circular trajectories), and swarm demos (pyramid formation, straight-line movement for N agents).

### bebop_gui
Qt graphical interfaces for the different demos.

### bebop_gz
Gazebo (gz-sim) assets and system plugins, built with `colcon` like any other ROS package: drone/world models and worlds, the `SetPosePlugin`/`RobotPosePublisher` gz-sim plugins, and a parametric world/bridge-config generator (`world_generator.py`) that adapts to N agents.

### bebop_ros_gz
Meta-package integrating all of the above — the complete interface for using ROS 2 with Gazebo simulation.
