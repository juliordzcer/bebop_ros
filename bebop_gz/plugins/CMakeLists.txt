cmake_minimum_required(VERSION 3.10)
project(SetPosePlugin)

# Buscar las dependencias de Gazebo Sim 9
find_package(gz-sim9 REQUIRED)
find_package(gz-transport14 REQUIRED)  # Cambiado de gz-transport12 a gz-transport14
find_package(gz-msgs11 REQUIRED)       # Asegúrate de usar la versión correcta de gz-msgs
find_package(gz-math8 REQUIRED)        # Asegúrate de usar la versión correcta de gz-math
find_package(gz-plugin3 REQUIRED)      # Asegúrate de usar la versión correcta de gz-plugin

# Crear la biblioteca del plugin
add_library(SetPosePlugin SHARED SetPosePlugin.cc)

# Enlazar las bibliotecas necesarias
target_link_libraries(SetPosePlugin
  gz-sim9::gz-sim9
  gz-transport14::gz-transport14  # Cambiado de gz-transport12 a gz-transport14
  gz-msgs11::gz-msgs11            # Asegúrate de usar la versión correcta de gz-msgs
  gz-math8::gz-math8              # Asegúrate de usar la versión correcta de gz-math
  gz-plugin3::gz-plugin3          # Asegúrate de usar la versión correcta de gz-plugin
)

# Instalar el plugin en la carpeta correcta
install(TARGETS SetPosePlugin DESTINATION lib)