cmake_minimum_required(VERSION 3.0.2)
project(project)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-10.2)


find_package(CUDA REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(Eigen3 3.3.9 REQUIRED)

#set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -DCLI11_BOOST_OPTIONAL=0")

include_directories(${PCL_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})

CUDA_ADD_LIBRARY(project_lib STATIC project.cu project.h)

target_link_libraries(project_lib ${PCL_LIBRARIES})
