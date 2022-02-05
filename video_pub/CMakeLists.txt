cmake_minimum_required(VERSION 2.8.3)
project(video_pub)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)
set(ROS_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Release)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
find_package( OpenCV 4 REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES windMill
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
 include/
  ${catkin_INCLUDE_DIRS}
)

# cuda_add_executable(${PROJECT_NAME}_node
#     src/video_pub.cpp
#     src/preprocess_kernel.cu
#     src/preprocess.cpp
#     )

 add_executable(${PROJECT_NAME}_node
     src/video_pub.cpp
     )
     
 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
     )

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
