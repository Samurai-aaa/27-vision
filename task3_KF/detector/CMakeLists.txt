cmake_minimum_required(VERSION 3.16)
project(detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(armor_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# OpenCV（cv_bridge 自带，但节点直接用也显式找）
find_package(OpenCV REQUIRED)
# OpenVINO：编译前需先加载环境变量：
#   source /opt/intel/openvino_2024.6.0/setupvars.sh
find_package(OpenVINO REQUIRED)

add_executable(detector_node
  src/detector_node.cpp
  src/OpenvinoInfer.cpp
  src/armor.cpp
  src/solver.cpp
)
target_include_directories(detector_node PRIVATE include)
ament_target_dependencies(detector_node
  rclcpp sensor_msgs cv_bridge armor_interfaces ament_index_cpp
  visualization_msgs rcl_interfaces
)
target_link_libraries(detector_node ${OpenCV_LIBS} openvino::runtime)

# video_player_node：源文件就绪后自动纳入构建
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/video_player_node.cpp")
  add_executable(video_player_node src/video_player_node.cpp)
  ament_target_dependencies(video_player_node rclcpp sensor_msgs cv_bridge)
  target_link_libraries(video_player_node ${OpenCV_LIBS})
  list(APPEND DETECTOR_TARGETS video_player_node)
endif()

install(TARGETS detector_node ${DETECTOR_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

# 模型随包安装，节点用 ament_index_cpp 定位 share 目录，摆脱相对路径依赖
install(DIRECTORY Model DESTINATION share/${PROJECT_NAME})

# launch 文件随包安装
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
