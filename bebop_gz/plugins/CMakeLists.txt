cmake_minimum_required(VERSION 3.10)
project(bebop_gz_plugins)

# Gazebo (gz-sim) ships versioned CMake config files (e.g. gz-cmake3,
# gz-sim8, gz-transport13, ...) rather than unversioned ones, so the
# find_package() calls must reference the exact major version that is
# installed. These versions correspond to Gazebo Harmonic, the release
# paired with ROS 2 Jazzy/Kilted (see README.md for install instructions).
# If you are on a different Gazebo release, adjust the suffixes below to
# match `dpkg -l | grep libgz-cmake` on your system.
find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-math7 REQUIRED)
find_package(gz-common5 REQUIRED)

set(GZ_PLUGIN_LIBS
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-plugin2::register
  gz-math7::gz-math7
  gz-common5::gz-common5
)

# Plugin 1: SetPosePlugin
add_library(SetPosePlugin SHARED SetPosePlugin.cc)
target_link_libraries(SetPosePlugin ${GZ_PLUGIN_LIBS})

# Plugin 2: RobotPosePublisher
add_library(RobotPosePublisher SHARED RobotPosePublisher.cc)
target_link_libraries(RobotPosePublisher ${GZ_PLUGIN_LIBS})

# Installed next to the other bebop_gz assets so a single
# GZ_SIM_SYSTEM_PLUGIN_PATH entry (set by the bebop_gz environment hook)
# is enough to find them.
install(
  TARGETS SetPosePlugin RobotPosePublisher
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
