cmake_minimum_required(VERSION 3.16)
project(tracker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(armor_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)   # 仅 tracker_node 2D 渲染用
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# 算法层静态库：ekf / target / simple_target / tracker 纯 C++ 不依赖 ROS
add_library(tracker_core STATIC
  src/extended_kalman_filter.cpp
  src/simple_target.cpp
  src/target.cpp
  src/tracker.cpp
)
target_include_directories(tracker_core PUBLIC include)
target_link_libraries(tracker_core PUBLIC Eigen3::Eigen)

# tracker_node（ROS 壳）：源文件就绪后自动纳入构建（对齐 detector 包风格）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/tracker_node.cpp")
  add_executable(tracker_node src/tracker_node.cpp)
  target_include_directories(tracker_node PRIVATE include)
  target_link_libraries(tracker_node tracker_core ${OpenCV_LIBS})
  ament_target_dependencies(
    tracker_node rclcpp armor_interfaces visualization_msgs geometry_msgs
    sensor_msgs cv_bridge)
  install(TARGETS tracker_node DESTINATION lib/${PROJECT_NAME})
endif()

# simple_tracker_node（需求①② 历史单板 KF 的 ROS 壳，独立 /simple_tracker/* 话题）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/simple_tracker_node.cpp")
  add_executable(simple_tracker_node src/simple_tracker_node.cpp)
  target_include_directories(simple_tracker_node PRIVATE include)
  target_link_libraries(simple_tracker_node tracker_core ${OpenCV_LIBS})
  ament_target_dependencies(
    simple_tracker_node rclcpp armor_interfaces sensor_msgs cv_bridge)
  install(TARGETS simple_tracker_node DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY include/ DESTINATION include)

# 参数文件 + launch 文件随包安装（ros2 launch tracker tracker.launch.py 用）
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
