cmake_minimum_required(VERSION 3.16)
project(scorpion_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY meshes launch DESTINATION share/${PROJECT_NAME})
install(FILES scorpion_ros.urdf scorpion.urdf scorpion.srdf collision_config.json initial_state.json DESTINATION share/${PROJECT_NAME})
install(FILES
  variants/wrist_cameras/scorpion_wrist_cameras.urdf
  variants/wrist_cameras/scorpion_wrist_cameras_ros.urdf
  variants/wrist_cameras/scorpion_wrist_cameras.srdf
  DESTINATION share/${PROJECT_NAME}/variants/wrist_cameras)
ament_package()
