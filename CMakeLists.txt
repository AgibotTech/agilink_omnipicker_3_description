cmake_minimum_required(VERSION 3.8)
project(omnipicker_3_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  urdf
  meshes
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md README_EN.md DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/gripper_control.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
