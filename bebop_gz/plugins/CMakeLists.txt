cmake_minimum_required(VERSION 3.10)
project(GazeboPlugins)

# Encuentra las dependencias de Gazebo
find_package(gz-cmake REQUIRED)  # Asegúrate de incluir gz-cmake
find_package(gz-sim REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz-plugin REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz-common REQUIRED)

# Plugin 1: SetPosePlugin
add_library(SetPosePlugin SHARED SetPosePlugin.cc)
target_link_libraries(SetPosePlugin
  gz-sim::gz-sim
  gz-transport::gz-transport
  gz-msgs::gz-msgs
  gz-plugin::gz-plugin
  gz-math::gz-math
  gz-common::gz-common
)

# Plugin 2: RobotPosePublisher
add_library(RobotPosePublisher SHARED RobotPosePublisher.cc)
target_link_libraries(RobotPosePublisher
  gz-sim::gz-sim
  gz-transport::gz-transport
  gz-plugin::gz-plugin
  gz-math::gz-math
  gz-common::gz-common
)

# Instalar los plugins
install(TARGETS SetPosePlugin RobotPosePublisher DESTINATION lib)
