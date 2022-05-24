cmake_minimum_required(VERSION 2.8.3)
project(mv_driver)


add_compile_options(-std=c++17)
message(WARNING, "  Linux, this is your birthday cake, there are butter, egges, water, flour. You TM compile it yourself!")
#set(ROS_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Debug)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
 cv_bridge
 image_transport
 roscpp
 sensor_msgs
 std_msgs
# ros_dynamic_test
)
find_package( OpenCV 4 REQUIRED )
catkin_package()
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
MESSAGE( STATUS "catkin_INCLUDE_DIRS = ${catkin_INCLUDE_DIRS}.")
set(ROS_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Release)

add_executable(${PROJECT_NAME}_node
    src/MVCamera_node.cpp
    src/MVCamera.cpp
    )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )
## Specify libraries to link a library or executable target against
if( ${ARCHITECTURE} STREQUAL "x86_64" )
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
${PROJECT_SOURCE_DIR}/lib/x64/libMVSDK.so
)
else()
   target_link_libraries(${PROJECT_NAME}_node
     ${catkin_LIBRARIES}
     ${OpenCV_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/lib/arm64/libMVSDK.so
   )
endif()
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
message("project path: ${PROJECT_SOURCE_DIR}")
