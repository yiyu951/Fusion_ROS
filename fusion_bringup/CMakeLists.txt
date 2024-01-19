cmake_minimum_required(VERSION 3.8)
project(fusion_bringup)

find_package(ament_cmake REQUIRED)
find_package(dataloader REQUIRED)
find_package(eskf REQUIRED)

find_package(fusion_interfaces REQUIRED)
find_package(fusion_node REQUIRED)

find_package(fusion_ros REQUIRED)

install(DIRECTORY 
  launch config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
