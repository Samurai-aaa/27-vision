# ROS binary cv_bridge uses distro OpenCV. Keep task3 isolated from /usr/local.
set(TASK3_OPENCV_DIR "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/opencv4"
    CACHE PATH "OpenCV config matching the selected cv_bridge build")
if(NOT EXISTS "${TASK3_OPENCV_DIR}/OpenCVConfig.cmake")
  message(FATAL_ERROR "Set TASK3_OPENCV_DIR to the OpenCV config used to build cv_bridge")
endif()
set(OpenCV_DIR "${TASK3_OPENCV_DIR}" CACHE PATH "Consistent task3 OpenCV" FORCE)
find_package(OpenCV REQUIRED CONFIG)
set(TASK3_SELECTED_OPENCV_VERSION "${OpenCV_VERSION}")
find_package(cv_bridge REQUIRED)
if(NOT OpenCV_VERSION VERSION_EQUAL TASK3_SELECTED_OPENCV_VERSION)
  message(FATAL_ERROR "cv_bridge exports OpenCV ${OpenCV_VERSION}, nodes selected ${TASK3_SELECTED_OPENCV_VERSION}; rebuild a consistent stack")
endif()
message(STATUS "task3 OpenCV ${TASK3_SELECTED_OPENCV_VERSION}: ${OpenCV_DIR}")
