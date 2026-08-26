cmake_minimum_required(VERSION 3.16)
project(armor_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 纯 OpenCV，不依赖 ROS2
find_package(OpenCV REQUIRED)

add_executable(armor_detector
  src/main.cpp
  src/detector.cpp
)

target_include_directories(armor_detector PRIVATE include)

target_link_libraries(armor_detector ${OpenCV_LIBS})
