cmake_minimum_required(VERSION 3.0.2)
project(small_map)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        radar_msgs
        std_msgs
        cv_bridge
        image_transport
        sensor_msgs
        )
find_package(OpenCV 4 REQUIRED)
catkin_package(

)

include_directories(
        ${catkin_INCLUDE_DIRS}
        "${CATKIN_DEVEL_PREFIX}/include"
#        "/usr/include/opencv2"
)
message("include dirs:${catkin_INCLUDE_DIRS}")
add_compile_options(-std=c++17)
add_executable(${PROJECT_NAME}
        src/small_map.cpp
        )

add_dependencies(${PROJECT_NAME} radar_msgs_generate_messages)#调用同一工作空间的自定义消息类型时注明依赖关系，防止发生头文件找不到的报错

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )