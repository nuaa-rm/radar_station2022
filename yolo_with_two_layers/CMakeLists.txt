cmake_minimum_required(VERSION 3.0.2)
project(yolo_with_two_layers)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  mv_driver
  rospy
  sensor_msgs
  image_transport
  cv_bridge
  std_msgs
)
catkin_package(
        DEPENDS
)
add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)
add_definitions(-DPACK_PATH="${PROJECT_SOURCE_DIR}")
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
#OPENCV
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
message('OPENCV FOUND:${OpenCV_FOUND}')
# cuda
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-10.2)
find_package(CUDA REQUIRED)
message('CUDA FOUND:${CUDA_FOUND}')
## Specify additional locations of header files
## Your package locations should be listed before other locations
string(REGEX REPLACE "(.+)\\src/yolo.*" "\\1" PROJECT_INIT_PATH ${PROJECT_SOURCE_DIR})
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        "/usr/include/eigen3"
        ../../../devel/include/
)# include and link dirs of cuda and tensorrt, you need adapt them if yours are different

include_directories(${CUDA_INCLUDE_DIRS})
message('CUDA_LIBRARIES:${CUDA_INCLUDE_DIRS}')
link_directories(/usr/local/cuda-10.2/lib64)
#include_directories(include)
# tensorrt

include_directories(/home/chris/TensorRT-7.0.0.11/include/)
link_directories(/home/chris/TensorRT-7.0.0.11/lib/)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -g -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")
cuda_add_library(myplugins2 SHARED src/yololayer.cu)
target_link_libraries(myplugins2 nvinfer cudart)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
cuda_add_executable(yolo_with_two_layers_node src/calibrator.cpp src/yolov5.cpp src/preprocess.cu)
add_dependencies(yolo_with_two_layers_node mv_driver_node)
target_link_libraries(yolo_with_two_layers_node nvinfer)
target_link_libraries(yolo_with_two_layers_node cudart)
target_link_libraries(yolo_with_two_layers_node myplugins2)
target_link_libraries(yolo_with_two_layers_node ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        )


