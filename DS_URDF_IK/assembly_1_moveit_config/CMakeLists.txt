cmake_minimum_required(VERSION 3.22)
project(assembly_1_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS
    scripts/fake_follow_joint_trajectory.py
    scripts/follower_follow_joint_trajectory.py
    scripts/isaac_follow_joint_trajectory.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/create_isaac_moveit_graph.py DESTINATION share/${PROJECT_NAME}/scripts)
install(
  FILES .setup_assistant README_FOLLOWER.md README_ISAAC.md README_ISAAC_FOLLOWER.md
  DESTINATION share/${PROJECT_NAME})
