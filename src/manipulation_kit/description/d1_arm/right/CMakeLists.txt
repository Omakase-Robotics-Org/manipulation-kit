cmake_minimum_required(VERSION 3.8)
project(d1_arm_right)

find_package(ament_cmake REQUIRED)

install(
  FILES
    d1_arm_right.urdf
    d1_arm_right_with_yubi.urdf.xacro
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
