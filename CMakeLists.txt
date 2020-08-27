cmake_minimum_required(VERSION 2.8)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

project(main)

set( CMAKE_CXX_FLAGS "-std=c++11") 

#寻找opencv库
find_package(OpenCV REQUIRED)
find_package(Qt5Core )

#添加opencv头文件
include_directories(${Opencv_INCLUDE_DIRS} LaneDetector.h)

add_library(LaneDetector_shared SHARED LaneDetector.cpp)

add_executable(lane main.cpp)
#链接库
target_link_libraries(lane ${OpenCV_LIBS} LaneDetector_shared Qt5::Core) 


