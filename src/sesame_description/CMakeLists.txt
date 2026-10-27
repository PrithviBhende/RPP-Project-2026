cmake_minimum_required(VERSION 3.8)
project(sesame_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    meshes
    urdf
    worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    model.config
    model.sdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
