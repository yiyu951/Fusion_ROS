cmake_minimum_required(VERSION 3.8)
project(eskf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)



find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)
find_package(dataloader REQUIRED)

find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(fusion_interfaces REQUIRED)

## Eskf
add_library(${PROJECT_NAME} SHARED
        src/eskf.cpp
        src/imu_init.cpp
        include/eskf/eskf.hpp
        include/eskf/imu_init.hpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(${PROJECT_NAME}
        fmt::fmt
        Sophus
)
ament_target_dependencies(${PROJECT_NAME}
        dataloader
)


install(TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

## Eskf Node

add_executable(${PROJECT_NAME}_node
        src/eskf_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
        eskf
)
ament_target_dependencies(${PROJECT_NAME}_node
        fusion_interfaces
        rclcpp
        nav_msgs std_msgs geometry_msgs sensor_msgs
        tf2 tf2_ros
)

install(
        TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)

    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
