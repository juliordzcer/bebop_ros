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
  gz-plugin3::gz-plugin3
  gz-math8::gz-math8
  gz-common6::gz-common6
)

# Plugin 3: Common
add_library(Common SHARED Common.cc)
target_link_libraries(Common
  gz-sim9::gz-sim9
  gz-math8::gz-math8
  gz-common6::gz-common6
)

# Plugin 4: LeeVelocityController
add_library(LeeVelocityController SHARED LeeVelocityController.cc)
target_link_libraries(LeeVelocityController
  gz-sim9::gz-sim9
)

# Plugin 5: MulticopterVelocityControl
add_library(MulticopterVelocityControl SHARED MulticopterVelocityControl.cc)
target_link_libraries(MulticopterVelocityControl
  gz-sim9::gz-sim9
  gz-transport14::gz-transport14
  gz-msgs11::gz-msgs11
  gz-plugin3::gz-plugin3
  gz-math8::gz-math8
  gz-common6::gz-common6
)

# Plugin 6: VelocityControl
add_library(MulticopterMotorModel SHARED MulticopterMotorModel.cc)
target_link_libraries(MulticopterMotorModel
  gz-sim9::gz-sim9
  gz-transport14::gz-transport14
  gz-msgs11::gz-msgs11
  gz-plugin3::gz-plugin3
  gz-math8::gz-math8
  gz-common6::gz-common6
)

# Plugin 7: VelocityControl
add_library(VelocityControl SHARED VelocityControl.cc)
target_link_libraries(VelocityControl
  gz-sim9::gz-sim9
  gz-transport14::gz-transport14
  gz-msgs11::gz-msgs11
  gz-plugin3::gz-plugin3
  gz-math8::gz-math8
  gz-common6::gz-common6
)

# Plugin 8: VelocityControllerPID
add_library(VelocityControllerPID SHARED VelocityControllerPID.cc)
target_link_libraries(VelocityControllerPID
  gz-sim9::gz-sim9
  gz-transport14::gz-transport14
  gz-msgs11::gz-msgs11
  gz-plugin3::gz-plugin3
  gz-math8::gz-math8
  gz-common6::gz-common6
)

# Plugin 9: PIDController
add_library(PIDController SHARED PIDController.cc)
target_link_libraries(PIDController
  gz-sim9::gz-sim9
  gz-transport14::gz-transport14
  gz-msgs11::gz-msgs11
  gz-plugin3::gz-plugin3
  gz-math8::gz-math8
  gz-common6::gz-common6
)

# Instalar los plugins
install(TARGETS SetPosePlugin RobotPosePublisher Common LeeVelocityController MulticopterMotorModel MulticopterVelocityControl VelocityControl VelocityControllerPID PIDController DESTINATION lib)