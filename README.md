## Requirements

### Installing ROS Rolling on ubuntu 24.04
#### Set locale
```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

#### Enable required repositories
You will need to add the ROS 2 apt repository to your system.
First ensure that the Ubuntu Universe repository is enabled.
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```
Now add the ROS 2 GPG key with apt.
```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
Then add the repository to your sources list.
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
Install development tools
```
sudo apt update && sudo apt install -y \
  python3-flake8-blind-except \
  python3-flake8-class-newline \
  python3-flake8-deprecated \
  python3-mypy \
  python3-pip \
  python3-pytest \
  python3-pytest-cov \
  python3-pytest-mock \
  python3-pytest-repeat \
  python3-pytest-rerunfailures \
  python3-pytest-runner \
  python3-pytest-timeout \
  ros-dev-tools
```
#### Environment setup
```
echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
#### Dependencies for building packages
Up to now you have installed what you need to run the core ROS packages. To create and manage your own ROS workspaces, there are various tools and requirements that are distributed separately. For example, rosinstall is a frequently used command-line tool that enables you to easily download many source trees for ROS packages with one command.

To install this tool and other dependencies for building ROS packages, run:
```
sudo apt install python3-rosdep python3-rosinstall-generator build-essential
```
##### Initialize rosdep
Before you can use many ROS tools, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS. If you have not yet installed rosdep, do so as follows.
```
sudo apt install python3-rosdep
```
With the following, you can initialize rosdep.
```
sudo rosdep init
rosdep update
```

### Install dependencies.
To be able to execute the programs it is necessary to install the following dependencies, executing the following commands in the console
```
sudo apt-get install ros-rolling-desktop -y
sudo apt-get install ros-rolling-ros-gz -y
sudo apt-get install ros-rolling-joy -y
sudo apt-get install ros-rolling-tf-transformations -y
sudo apt-get install ros-rolling-ament-lint-auto -y
sudo apt-get install ros-rolling-ament-cmake -y
sudo apt-get install cmake -y
```
```
sudo pip3 install transforms3d --break-system-packages
sudo pip3 install numpy --break-system-packages 
sudo pip3 install matplotlib --break-system-packages
sudo pip3 install scipy --break-system-packages
sudo pip3 install empy --break-system-packages
```

## Install Gazebo Jetty
First install some necessary tools:
```
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
```
Then install Gazebo jetty:
```
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-prerelease $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-prerelease.list > /dev/null
sudo apt-get update
sudo apt-get install gz-jetty
```
```
sudo apt-get install libgz-cmake3-dev -y
```
## Create a catkin workspace
```
mkdir -p ~/ws_bebop/src
cd ws_bebop/src
```
With these commands a new workspace called `ws_bebop` has been created


## Installation

Clone the package into your catkin workspace (in src folder): 
```
cd
cd ws_bebop/src
git clone --recursive https://github.com/juliordzcer/bebop_ros.git
cd bebop_ros/bebop_gz/plugins/build
```
```
cmake ..
```

```
make
```


Use `colcon build` on your workspace to compile.
```
cd ws_bebop
colcon build
```
## **Setup Instructions**

### **Add Environment Variables and Source Setup File**

You can add the necessary environment variables and source file to your `.bashrc` file using the following commands:

```bash
echo "source ~/ws_bebop/install/setup.bash" >> ~/.bashrc
echo "export GZ_SIM_RESOURCE_PATH=\$HOME/ws_bebop/src/bebop_ros/bebop_gz/worlds:$HOME/ws_bebop/src/bebop_ros/bebop_gz/models" >> ~/.bashrc
echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ws_bebop/src/bebop_ros/bebop_gz/plugins/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}" >> ~/.bashrc
echo "export GZ_VERSION=jetty" >> ~/.bashrc
source ~/.bashrc

```

### **Alternatively, Edit `.bashrc` Manually**

1. Open the `.bashrc` file with `nano`:

```bash
nano ~/.bashrc
```

2. Add the following lines at the end of the file:

```bash
source ~/ws_bebop/install/setup.bash
export GZ_SIM_RESOURCE_PATH="$HOME/ws_bebop/src/bebop_ros/bebop_gz/worlds:$HOME/ws_bebop/src/bebop_ros/bebop_gz/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ws_bebop/src/bebop_ros/bebop_gz/plugins/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export GZ_VERSION=jetty
```

3. Save the file by pressing `Ctrl+O`, then press `Enter`.
4. Exit `nano` by pressing `Ctrl+X`.
5. Finally, reload the `.bashrc`:

```bash
source ~/.bashrc
```

**The environment should now be configured correctly. You can proceed to run your Bebop2 drone simulations in Gazebo.**

## Package Overview
This project contains six main packages that work together to provide drone simulation capabilities:

### bebop_bearings
Implements a multi-agent bearing-based formation control system for 4 agents, achieving formation through bearing measurements.
**Demo command:**
```
ros2 launch bebop_bearings bebop.launch.py
```

### bebop_controller
Contains a position-based PID controller for single-agent trajectory tracking. Also includes an additional demo for swarm formation:
- Pyramid formation with leader-follower topology
- Straight-line movement for N agents

**Demo Commands:**
Single agent:
```
ros2 launch bebop_demo bebop1.launch.py
```

Swarm formation:

```
ros2 launch bebop_demo swarmbebop.launch.py
```

### bebop_demo
Contains utility packages and demonstration setups:
- `set_pose`: Sets initial drone positions
- `setpoint`: Generates circular trajectories for single drone operation
- Various executable examples

### bebop_gui
Contains graphical user interfaces for the different demos.

### bebop_gz
Gazebo-specific components including:
- Drone models and plugins
- Simulation worlds
- A configurable world generator that adapts to N agents

### bebop_ros_gz
Integrates all Gazebo packages into a single project. This meta-package provides the complete interface for using ROS 2 with Gazebo simulation.
