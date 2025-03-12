cmake_minimum_required(VERSION 3.10)
project(GazeboPlugins)

# Versiones específicas para Ignition Garden
find_package(gz-sim9 REQUIRED)
find_package(gz-transport14 REQUIRED)
find_package(gz-msgs11 REQUIRED)
find_package(gz-plugin3 REQUIRED)
find_package(gz-math8 REQUIRED)

add_library(SetPosePlugin SHARED SetPosePlugin.cc)
add_library(PPosePlugin SHARED PPosePlugin.cc)

target_link_libraries(SetPosePlugin
  gz-sim9::gz-sim9
  gz-transport14::gz-transport14
  gz-msgs11::gz-msgs11
  gz-plugin3::gz-plugin3
  gz-math8::gz-math8
)

target_link_libraries(PPosePlugin
  gz-sim9::gz-sim9
  gz-sim9::core
  gz-transport14::gz-transport14
  gz-msgs11::gz-msgs11
  gz-plugin3::gz-plugin3
  gz-math8::gz-math8
)

install(TARGETS SetPosePlugin PPosePlugin DESTINATION lib)