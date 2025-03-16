cmake_minimum_required(VERSION 3.10)
project(GazeboPlugins)

# Encuentra las dependencias de Gazebo
find_package(gz-cmake3 REQUIRED)  # Asegúrate de incluir gz-cmake
find_package(gz-sim9 REQUIRED)
find_package(gz-transport14 REQUIRED)
find_package(gz-msgs11 REQUIRED)
find_package(gz-plugin3 REQUIRED)
find_package(gz-math8 REQUIRED)
find_package(gz-common6 REQUIRED)

# Plugin 1: SetPosePlugin
add_library(SetPosePlugin SHARED SetPosePlugin.cc)
target_link_libraries(SetPosePlugin
  gz-sim9::gz-sim9
  gz-transport14::gz-transport14
  gz-msgs11::gz-msgs11
  gz-plugin3::gz-plugin3
  gz-math8::gz-math8
  gz-common6::gz-common6
)

# Plugin 2: RobotPosePublisher
add_library(RobotPosePublisher SHARED RobotPosePublisher.cc)
target_link_libraries(RobotPosePublisher
  gz-sim9::gz-sim9
  gz-transport14::gz-transport14
  gz-msgs11::gz-msgs11
  gz-plugin3::gz-plugin3
  gz-math8::gz-math8
  gz-common6::gz-common6
)

# Instalar los plugins
install(TARGETS SetPosePlugin RobotPosePublisher DESTINATION lib)