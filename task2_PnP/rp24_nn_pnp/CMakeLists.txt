cmake_minimum_required(VERSION 3.16)
project(rp_detect)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 纯 OpenCV + OpenVINO
find_package(OpenCV REQUIRED)

# OpenVINO：编译前需先加载环境变量，使 find_package(OpenVINO) 找到 CMake 配置：
#   source /opt/intel/openvino_2024.6.0/setupvars.sh
# setupvars 会设置 OPENVINO_DIR 指向 runtime/cmake
find_package(OpenVINO REQUIRED)

add_executable(rp_detect
  src/main.cpp
  src/OpenvinoInfer.cpp
  src/armor.cpp
  src/solver.cpp
)

# 独立研究小工具:yaw 一维优化(yaw_study)。算法层在 yawstudy 模块,
# yaw_study_main.cpp 只做采集/编排;与主程序共享检测/PnP 模块
add_executable(rp_yaw_study
  src/yaw_study_main.cpp
  src/yawstudy.cpp
  src/OpenvinoInfer.cpp
  src/armor.cpp
  src/solver.cpp
)

# 头文件在 include/（main.cpp 里 #include "OpenvinoInfer.h" 需要能找到）
target_include_directories(rp_detect PRIVATE include)
target_include_directories(rp_yaw_study PRIVATE include)

target_link_libraries(rp_detect
  ${OpenCV_LIBS}
  openvino::runtime
)
target_link_libraries(rp_yaw_study
  ${OpenCV_LIBS}
  openvino::runtime
)
