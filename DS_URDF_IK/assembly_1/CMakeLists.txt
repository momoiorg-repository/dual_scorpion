cmake_minimum_required(VERSION 3.8)
project(assembly_1)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf meshes rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
