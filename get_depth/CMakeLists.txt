cmake_minimum_required(VERSION 3.0.2)
project(get_depth)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        cv_bridge
        sensor_msgs
        radar_msgs
        tf
        )
find_package(PCL 1.2 REQUIRED)
find_package(OpenCV)
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES get_depth
        #  CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)
string(REGEX REPLACE "(.+)\\src/get_depth.*" "\\1" PROJECT_INIT_PATH ${PROJECT_SOURCE_DIR})
add_subdirectory(src/project)
include_directories(
        #         include
        ${catkin_INCLUDE_DIRS}
        OpenCV_INCLUDE_DIRS
        ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_executable(${PROJECT_NAME}_node src/get_depth_node.cpp)
add_dependencies(${PROJECT_NAME}_node radar_msgs_generate_messages)
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        project_lib
        )
