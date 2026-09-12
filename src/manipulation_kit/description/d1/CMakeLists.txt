cmake_minimum_required(VERSION 3.8)
project(d1_description)

find_package(ament_cmake REQUIRED)

install(
  FILES
    d1.urdf
    d1_wholebody.urdf
    d1_wholebody_gripper.urdf
  DESTINATION share/${PROJECT_NAME}
)

# d1_wholebody_gripper.urdf references these RELATIVE to itself, so they must
# land in the same layout under share/ for the installed copy to resolve.
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
