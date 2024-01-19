cmake_minimum_required(VERSION 3.8)
project(dataloader)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找并添加Eigen库
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# 寻找并添加Sophus库
find_package(Sophus REQUIRED)

find_package(fmt REQUIRED)

# ROS 添加消息依赖项
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(fusion_interfaces REQUIRED)


add_library(${PROJECT_NAME}
        src/dataloader.cpp
        include/dataloader/dataloader.hpp
        include/dataloader/imu.hpp
        include/dataloader/odom.hpp
        include/dataloader/pose.hpp
        include/dataloader/state.hpp
        include/dataloader/types.hpp
)


target_include_directories(${PROJECT_NAME} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
        fmt::fmt
        Sophus glog
)

ament_target_dependencies(${PROJECT_NAME}
        fusion_interfaces
        rclcpp
        nav_msgs std_msgs geometry_msgs sensor_msgs
        tf2 tf2_ros
)

## test
add_executable(analysis_data_test src/analysis_data_test.cpp)

target_link_libraries(analysis_data_test
        ${PROJECT_NAME}
)


install(
        TARGETS analysis_data_test
        DESTINATION lib/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}

        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME})

# 编译消息
ament_export_dependencies(rclcpp)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)

ament_export_dependencies(fusion_interfaces)

install(
        DIRECTORY include/ # ${CMAKE_CURRENT_BINARY_DIR}/include/
        DESTINATION include/${PROJECT_NAME}
)


#install(
#        TARGETS analysis_data_test
#        ARCHIVE DESTINATION lib
#        LIBRARY DESTINATION lib
#        RUNTIME DESTINATION bin
#)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

endif()

ament_package()
