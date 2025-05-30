# Install script for directory: /home/jr/ws_bebop/src/bebop_ros/bebop_gz/plugins

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libSetPosePlugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libSetPosePlugin.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libSetPosePlugin.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/jr/ws_bebop/src/bebop_ros/bebop_gz/plugins/build/libSetPosePlugin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libSetPosePlugin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libSetPosePlugin.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libSetPosePlugin.so"
         OLD_RPATH "/opt/ros/rolling/opt/gz_sim_vendor/lib:/opt/ros/rolling/opt/gz_fuel_tools_vendor/lib:/opt/ros/rolling/opt/gz_gui_vendor/lib:/opt/ros/rolling/opt/gz_plugin_vendor/lib:/opt/ros/rolling/opt/gz_physics_vendor/lib:/opt/ros/rolling/opt/gz_rendering_vendor/lib:/opt/ros/rolling/opt/gz_common_vendor/lib:/opt/ros/rolling/opt/gz_utils_vendor/lib:/opt/ros/rolling/opt/gz_transport_vendor/lib:/opt/ros/rolling/opt/gz_msgs_vendor/lib:/opt/ros/rolling/opt/sdformat_vendor/lib:/opt/ros/rolling/opt/gz_math_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libSetPosePlugin.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/jr/ws_bebop/src/bebop_ros/bebop_gz/plugins/build/CMakeFiles/SetPosePlugin.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libRobotPosePublisher.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libRobotPosePublisher.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libRobotPosePublisher.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/jr/ws_bebop/src/bebop_ros/bebop_gz/plugins/build/libRobotPosePublisher.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libRobotPosePublisher.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libRobotPosePublisher.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libRobotPosePublisher.so"
         OLD_RPATH "/opt/ros/rolling/opt/gz_sim_vendor/lib:/opt/ros/rolling/opt/gz_fuel_tools_vendor/lib:/opt/ros/rolling/opt/gz_gui_vendor/lib:/opt/ros/rolling/opt/gz_plugin_vendor/lib:/opt/ros/rolling/opt/gz_physics_vendor/lib:/opt/ros/rolling/opt/gz_rendering_vendor/lib:/opt/ros/rolling/opt/gz_common_vendor/lib:/opt/ros/rolling/opt/gz_utils_vendor/lib:/opt/ros/rolling/opt/gz_transport_vendor/lib:/opt/ros/rolling/opt/gz_msgs_vendor/lib:/opt/ros/rolling/opt/sdformat_vendor/lib:/opt/ros/rolling/opt/gz_math_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libRobotPosePublisher.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/jr/ws_bebop/src/bebop_ros/bebop_gz/plugins/build/CMakeFiles/RobotPosePublisher.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCommon.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCommon.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCommon.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/jr/ws_bebop/src/bebop_ros/bebop_gz/plugins/build/libCommon.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCommon.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCommon.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCommon.so"
         OLD_RPATH "/opt/ros/rolling/opt/gz_sim_vendor/lib:/opt/ros/rolling/opt/gz_fuel_tools_vendor/lib:/opt/ros/rolling/opt/gz_gui_vendor/lib:/opt/ros/rolling/opt/gz_plugin_vendor/lib:/opt/ros/rolling/opt/gz_physics_vendor/lib:/opt/ros/rolling/opt/gz_rendering_vendor/lib:/opt/ros/rolling/opt/gz_common_vendor/lib:/opt/ros/rolling/opt/gz_utils_vendor/lib:/opt/ros/rolling/opt/gz_transport_vendor/lib:/opt/ros/rolling/opt/gz_msgs_vendor/lib:/opt/ros/rolling/opt/sdformat_vendor/lib:/opt/ros/rolling/opt/gz_math_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libCommon.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/jr/ws_bebop/src/bebop_ros/bebop_gz/plugins/build/CMakeFiles/Common.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libLeeVelocityController.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libLeeVelocityController.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libLeeVelocityController.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/jr/ws_bebop/src/bebop_ros/bebop_gz/plugins/build/libLeeVelocityController.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libLeeVelocityController.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libLeeVelocityController.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libLeeVelocityController.so"
         OLD_RPATH "/opt/ros/rolling/opt/gz_sim_vendor/lib:/opt/ros/rolling/opt/gz_fuel_tools_vendor/lib:/opt/ros/rolling/opt/gz_gui_vendor/lib:/opt/ros/rolling/opt/gz_plugin_vendor/lib:/opt/ros/rolling/opt/gz_physics_vendor/lib:/opt/ros/rolling/opt/gz_rendering_vendor/lib:/opt/ros/rolling/opt/gz_common_vendor/lib:/opt/ros/rolling/opt/gz_utils_vendor/lib:/opt/ros/rolling/opt/gz_transport_vendor/lib:/opt/ros/rolling/opt/gz_msgs_vendor/lib:/opt/ros/rolling/opt/sdformat_vendor/lib:/opt/ros/rolling/opt/gz_math_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libLeeVelocityController.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/jr/ws_bebop/src/bebop_ros/bebop_gz/plugins/build/CMakeFiles/LeeVelocityController.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libMulticopterMotorModel.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libMulticopterMotorModel.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libMulticopterMotorModel.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/jr/ws_bebop/src/bebop_ros/bebop_gz/plugins/build/libMulticopterMotorModel.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libMulticopterMotorModel.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libMulticopterMotorModel.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libMulticopterMotorModel.so"
         OLD_RPATH "/opt/ros/rolling/opt/gz_sim_vendor/lib:/opt/ros/rolling/opt/gz_fuel_tools_vendor/lib:/opt/ros/rolling/opt/gz_gui_vendor/lib:/opt/ros/rolling/opt/gz_plugin_vendor/lib:/opt/ros/rolling/opt/gz_physics_vendor/lib:/opt/ros/rolling/opt/gz_rendering_vendor/lib:/opt/ros/rolling/opt/gz_common_vendor/lib:/opt/ros/rolling/opt/gz_utils_vendor/lib:/opt/ros/rolling/opt/gz_transport_vendor/lib:/opt/ros/rolling/opt/gz_msgs_vendor/lib:/opt/ros/rolling/opt/sdformat_vendor/lib:/opt/ros/rolling/opt/gz_math_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libMulticopterMotorModel.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/jr/ws_bebop/src/bebop_ros/bebop_gz/plugins/build/CMakeFiles/MulticopterMotorModel.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libMulticopterVelocityControl.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libMulticopterVelocityControl.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libMulticopterVelocityControl.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/jr/ws_bebop/src/bebop_ros/bebop_gz/plugins/build/libMulticopterVelocityControl.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libMulticopterVelocityControl.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libMulticopterVelocityControl.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libMulticopterVelocityControl.so"
         OLD_RPATH "/opt/ros/rolling/opt/gz_sim_vendor/lib:/opt/ros/rolling/opt/gz_fuel_tools_vendor/lib:/opt/ros/rolling/opt/gz_gui_vendor/lib:/opt/ros/rolling/opt/gz_plugin_vendor/lib:/opt/ros/rolling/opt/gz_physics_vendor/lib:/opt/ros/rolling/opt/gz_rendering_vendor/lib:/opt/ros/rolling/opt/gz_common_vendor/lib:/opt/ros/rolling/opt/gz_utils_vendor/lib:/opt/ros/rolling/opt/gz_transport_vendor/lib:/opt/ros/rolling/opt/gz_msgs_vendor/lib:/opt/ros/rolling/opt/sdformat_vendor/lib:/opt/ros/rolling/opt/gz_math_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libMulticopterVelocityControl.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/jr/ws_bebop/src/bebop_ros/bebop_gz/plugins/build/CMakeFiles/MulticopterVelocityControl.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libVelocityControl.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libVelocityControl.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libVelocityControl.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/jr/ws_bebop/src/bebop_ros/bebop_gz/plugins/build/libVelocityControl.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libVelocityControl.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libVelocityControl.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libVelocityControl.so"
         OLD_RPATH "/opt/ros/rolling/opt/gz_sim_vendor/lib:/opt/ros/rolling/opt/gz_fuel_tools_vendor/lib:/opt/ros/rolling/opt/gz_gui_vendor/lib:/opt/ros/rolling/opt/gz_plugin_vendor/lib:/opt/ros/rolling/opt/gz_physics_vendor/lib:/opt/ros/rolling/opt/gz_rendering_vendor/lib:/opt/ros/rolling/opt/gz_common_vendor/lib:/opt/ros/rolling/opt/gz_utils_vendor/lib:/opt/ros/rolling/opt/gz_transport_vendor/lib:/opt/ros/rolling/opt/gz_msgs_vendor/lib:/opt/ros/rolling/opt/sdformat_vendor/lib:/opt/ros/rolling/opt/gz_math_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libVelocityControl.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/jr/ws_bebop/src/bebop_ros/bebop_gz/plugins/build/CMakeFiles/VelocityControl.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libVelocityControllerPID.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libVelocityControllerPID.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libVelocityControllerPID.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/jr/ws_bebop/src/bebop_ros/bebop_gz/plugins/build/libVelocityControllerPID.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libVelocityControllerPID.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libVelocityControllerPID.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libVelocityControllerPID.so"
         OLD_RPATH "/opt/ros/rolling/opt/gz_sim_vendor/lib:/opt/ros/rolling/opt/gz_fuel_tools_vendor/lib:/opt/ros/rolling/opt/gz_gui_vendor/lib:/opt/ros/rolling/opt/gz_plugin_vendor/lib:/opt/ros/rolling/opt/gz_physics_vendor/lib:/opt/ros/rolling/opt/gz_rendering_vendor/lib:/opt/ros/rolling/opt/gz_common_vendor/lib:/opt/ros/rolling/opt/gz_utils_vendor/lib:/opt/ros/rolling/opt/gz_transport_vendor/lib:/opt/ros/rolling/opt/gz_msgs_vendor/lib:/opt/ros/rolling/opt/sdformat_vendor/lib:/opt/ros/rolling/opt/gz_math_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libVelocityControllerPID.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/jr/ws_bebop/src/bebop_ros/bebop_gz/plugins/build/CMakeFiles/VelocityControllerPID.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libPIDController.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libPIDController.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libPIDController.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/jr/ws_bebop/src/bebop_ros/bebop_gz/plugins/build/libPIDController.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libPIDController.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libPIDController.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libPIDController.so"
         OLD_RPATH "/opt/ros/rolling/opt/gz_sim_vendor/lib:/opt/ros/rolling/opt/gz_fuel_tools_vendor/lib:/opt/ros/rolling/opt/gz_gui_vendor/lib:/opt/ros/rolling/opt/gz_plugin_vendor/lib:/opt/ros/rolling/opt/gz_physics_vendor/lib:/opt/ros/rolling/opt/gz_rendering_vendor/lib:/opt/ros/rolling/opt/gz_common_vendor/lib:/opt/ros/rolling/opt/gz_utils_vendor/lib:/opt/ros/rolling/opt/gz_transport_vendor/lib:/opt/ros/rolling/opt/gz_msgs_vendor/lib:/opt/ros/rolling/opt/sdformat_vendor/lib:/opt/ros/rolling/opt/gz_math_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libPIDController.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/jr/ws_bebop/src/bebop_ros/bebop_gz/plugins/build/CMakeFiles/PIDController.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/jr/ws_bebop/src/bebop_ros/bebop_gz/plugins/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
