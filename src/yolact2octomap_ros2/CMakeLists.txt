cmake_minimum_required(VERSION 3.5)
project(yolact2octomap_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

set(dependencies
    rclcpp
    std_msgs
    tf2_geometry_msgs
    tf2_ros
    tf2
    octomap
    octomap_msgs
)

include_directories(include)

add_executable(yolact2octomap_node src/yolact2octomap_node.cpp)
ament_target_dependencies(yolact2octomap_node ${dependencies})
target_link_libraries(yolact2octomap_node octomap)


install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  yolact2octomap_node  
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
