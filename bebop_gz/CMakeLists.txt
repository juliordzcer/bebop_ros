cmake_minimum_required(VERSION 3.10)
project(bebop_gz)

find_package(ament_cmake REQUIRED)

# Builds SetPosePlugin / RobotPosePublisher (see plugins/CMakeLists.txt for
# the Gazebo version notes) into lib/bebop_gz_plugins.
add_subdirectory(plugins)

# Drone/world models and worlds, installed under share/bebop_gz so they can
# be found regardless of where the workspace lives.
install(
  DIRECTORY models worlds
  DESTINATION share/${PROJECT_NAME}
)

# Parametric world/bridge-config generator, invoked from the demo launch
# files via the installed share directory (see get_package_share_directory
# calls in bebop_bearings/bebop_demo launch files).
install(
  PROGRAMS world_generator.py
  DESTINATION share/${PROJECT_NAME}
)

# Exports GZ_SIM_RESOURCE_PATH (models/worlds) and GZ_SIM_SYSTEM_PLUGIN_PATH
# (compiled plugins) automatically when install/setup.bash is sourced, so no
# manual .bashrc editing is required.
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/bebop_gz.sh.in"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
