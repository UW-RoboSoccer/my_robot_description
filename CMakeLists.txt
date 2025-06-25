cmake_minimum_required(VERSION 3.10)
project(my_robot_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY urdf mjcf meshes config launch v1_urdf v1_mjcf v1_meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
