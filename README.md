![image](https://github.com/DoveJH/readme_source/blob/main/2020radar_station/%E6%97%A0%E8%83%8C%E6%99%AF2.png)

# 长空御风雷达站

基于工业相机和Livox激光雷达的RoboMaster雷达站项目
由南京航空航天大学长空御风战队开源

## 项目介绍
本项目为南京航空航天大学长空御风战队的RoboMaster2022雷达站开源项目。
![image](https://github.com/DoveJH/readme_source/blob/main/2020radar_station/liucheng.png)
//机器人参数
//项目机构图

## 系统架构

### 系统层级
整个雷达站采用ROS（机器人操作系统）开发，代码遵循ROS架构。分为4层：硬件驱动层，数据处理层，功能逻辑层和前端显示层。

### 第三方中间件及模块:
CUDA_10.2, TensorRT, Tensorrtx, yolov5-6.0，OpenCV-4.5.4, Cmake，Qt5.9
另一些驱动程序，以ROS功能包的形式运行：Livox_ROS_Driver, MindVision_ROS_Driver, RealSense_ROS_Driver,ROS_Serial

### 开发及运行环境：
Ubuntu20.04(操作系统), CLion(软件开发环境)
*在Ubuntu18.04下程序仍可正常运行，但是性能开销明显上涨。*

## 部署

### 安装依赖项

 - ROS *（仅在Melodic和Noetic版本上进行过测试）*
 - Tensorrt
 - CUDA
 - OpenCV4
 - Qt5
 - PCL
 - Eigen
 - ros-[veision]-serial

### 配置工作

 - 根据https://github.com/ultralytics/yolov5下的引导完成神经网络的训练。
 - 根据https://github.com/wang-xinyu/tensorrtx下的指示生成.engine文件。
 - 将用于识别车辆和装甲板的engine文件分别命名为yolov5s_car.engine和yolov5s_number.engine，并放入yolo_with_two_layers包目录下。
 - **根据你使用tensorrtx生成engine时的配置，修改yolo_with_two_layers包中相应文件参数**，两套配置已通过变量命名的后缀指出。
 - 如果你使用的不是迈德威视工业相机，请启动你的相机节点。并将相机发布的话题修改为

### 编译
下载工程至你的工作空间下

    git clone https://github.com/nuaa-rm/radar_station2022.git

进入你的工作空间下

项目运行时，需要启动livox_ros_driver，为了以后无需额外source livox_ros_driver工程，需要在**编译前**执行以下步骤。如果已经编译，则可以删除编译生成文件，从新执行下述命令，并进行编译。

    source [你的livox_ros_driver工作空间]/devel/setup.bash

由于radar_msgs包和hp_limit_helper包中自定义了ROS的message类型，需要先行编译，否则会导致报错。

    catkin_make -DCATKIN_WHITELIST_PACKAGE="radar_msgs;hp_limit_helper"

然后编译其余包：

    catkin_make -DCATKIN_WHITELIST_PACKAGE=""

### 运行

    cd [workspace]
    source devel/setup.bash
    roslaunch radar_msgs radar.launch
如果你获得如下效果，即为成功运行。

## How It Works
![image](https://github.com/DoveJH/readme_source/blob/main/2020radar_station/gongneng.png)

##项目结构
├── radar_station2022
│   ├── 大范围pnp正投影.png
│   ├── 矩阵乘法优化后性能分析-perf.png
│   ├── calibrateCamera
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── src
│   │       └── calibrateCamera_node.cpp
│   ├── CMakeLists.txt -> /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
│   ├── dart_detect
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── src
│   │       └── dart_detect_node.cpp
│   ├── detect_mineral
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── src
│   │       └── detect_mineral_node.cpp
│   ├── displayer_app
│   │   ├── cfg
│   │   │   └── RadarDisplayerRos.yaml
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   └── displayer.launch
│   │   ├── node_modules
│   │   ├── package.xml
│   │   ├── README.md
│   │   ├── requirements.txt
│   │   ├── src
│   │   │   ├── app.py
│   │   │   ├── base.py
│   │   │   ├── config.py
│   │   │   ├── __init__.py
│   │   │   ├── main.py
│   │   │   ├── rosNode.py
│   │   │   ├── static
│   │   │   │   └── noImage.png
│   │   │   └── views
│   │   │       ├── camera.py
│   │   │       ├── httpApi.py
│   │   │       ├── __init__.py
│   │   │       ├── webView.py
│   │   │       └── wsApi.py
│   │   └── yarn.lock
│   ├── displayer_qt5
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── displayer_qt5
│   │   │       ├── main_window.hpp
│   │   │       ├── qlabel_with_mouse_event.h
│   │   │       ├── qlabel_with_painter.h
│   │   │       └── qnode.hpp
│   │   ├── mainpage.dox
│   │   ├── package.xml
│   │   ├── recorded.avi
│   │   ├── resources
│   │   │   ├── images
│   │   │   │   ├── blue_minimap.png
│   │   │   │   ├── CKYF.png
│   │   │   │   ├── CKYF.svg
│   │   │   │   ├── Icon.ico
│   │   │   │   ├── icon.png
│   │   │   │   ├── radar_logo.png
│   │   │   │   └── red_minimap.png
│   │   │   ├── images.qrc
│   │   │   └── qss
│   │   │       ├── progressBarBlue.qss
│   │   │       └── progressBarRed.qss
│   │   ├── src
│   │   │   ├── main.cpp
│   │   │   ├── main_window.cpp
│   │   │   ├── qlabel_with_mouse_event.cpp
│   │   │   ├── qlabel_with_painter.cpp
│   │   │   └── qnode.cpp
│   │   ├── ui
│   │   │   └── main_window.ui
│   │   └── yaml
│   │       ├── displayer_qt5.yaml
│   │       └── load_params.launch
│   ├── displayer_web
│   │   ├── 040010420186.mvdat
│   │   ├── build
│   │   │   ├── atomic_configure
│   │   │   │   ├── env.sh
│   │   │   │   ├── local_setup.bash
│   │   │   │   ├── local_setup.sh
│   │   │   │   ├── local_setup.zsh
│   │   │   │   ├── setup.bash
│   │   │   │   ├── setup.sh
│   │   │   │   ├── _setup_util.py
│   │   │   │   └── setup.zsh
│   │   │   ├── catkin
│   │   │   │   └── catkin_generated
│   │   │   │       └── version
│   │   │   │           └── package.cmake
│   │   │   ├── catkin_generated
│   │   │   │   ├── env_cached.sh
│   │   │   │   ├── generate_cached_setup.py
│   │   │   │   ├── installspace
│   │   │   │   │   ├── env.sh
│   │   │   │   │   ├── local_setup.bash
│   │   │   │   │   ├── local_setup.sh
│   │   │   │   │   ├── local_setup.zsh
│   │   │   │   │   ├── setup.bash
│   │   │   │   │   ├── setup.sh
│   │   │   │   │   ├── _setup_util.py
│   │   │   │   │   └── setup.zsh
│   │   │   │   ├── order_packages.cmake
│   │   │   │   ├── order_packages.py
│   │   │   │   ├── setup_cached.sh
│   │   │   │   └── stamps
│   │   │   │       └── Project
│   │   │   │           ├── interrogate_setup_dot_py.py.stamp
│   │   │   │           ├── order_packages.cmake.em.stamp
│   │   │   │           ├── package.xml.stamp
│   │   │   │           └── _setup_util.py.stamp
│   │   │   ├── CATKIN_IGNORE
│   │   │   ├── catkin_make.cache
│   │   │   ├── CMakeCache.txt
│   │   │   ├── CMakeFiles
│   │   │   │   ├── 3.10.2
│   │   │   │   │   ├── CMakeCCompiler.cmake
│   │   │   │   │   ├── CMakeCXXCompiler.cmake
│   │   │   │   │   ├── CMakeDetermineCompilerABI_C.bin
│   │   │   │   │   ├── CMakeDetermineCompilerABI_CXX.bin
│   │   │   │   │   ├── CMakeSystem.cmake
│   │   │   │   │   ├── CompilerIdC
│   │   │   │   │   │   ├── a.out
│   │   │   │   │   │   └── CMakeCCompilerId.c
│   │   │   │   │   └── CompilerIdCXX
│   │   │   │   │       ├── a.out
│   │   │   │   │       └── CMakeCXXCompilerId.cpp
│   │   │   │   ├── clean_test_results.dir
│   │   │   │   │   ├── build.make
│   │   │   │   │   ├── cmake_clean.cmake
│   │   │   │   │   ├── DependInfo.cmake
│   │   │   │   │   └── progress.make
│   │   │   │   ├── cmake.check_cache
│   │   │   │   ├── CMakeDirectoryInformation.cmake
│   │   │   │   ├── CMakeError.log
│   │   │   │   ├── CMakeOutput.log
│   │   │   │   ├── CMakeRuleHashes.txt
│   │   │   │   ├── download_extra_data.dir
│   │   │   │   │   ├── build.make
│   │   │   │   │   ├── cmake_clean.cmake
│   │   │   │   │   ├── DependInfo.cmake
│   │   │   │   │   └── progress.make
│   │   │   │   ├── doxygen.dir
│   │   │   │   │   ├── build.make
│   │   │   │   │   ├── cmake_clean.cmake
│   │   │   │   │   ├── DependInfo.cmake
│   │   │   │   │   └── progress.make
│   │   │   │   ├── feature_tests.bin
│   │   │   │   ├── feature_tests.c
│   │   │   │   ├── feature_tests.cxx
│   │   │   │   ├── Makefile2
│   │   │   │   ├── Makefile.cmake
│   │   │   │   ├── progress.marks
│   │   │   │   ├── run_tests.dir
│   │   │   │   │   ├── build.make
│   │   │   │   │   ├── cmake_clean.cmake
│   │   │   │   │   ├── DependInfo.cmake
│   │   │   │   │   └── progress.make
│   │   │   │   ├── TargetDirectories.txt
│   │   │   │   └── tests.dir
│   │   │   │       ├── build.make
│   │   │   │       ├── cmake_clean.cmake
│   │   │   │       ├── DependInfo.cmake
│   │   │   │       └── progress.make
│   │   │   ├── cmake_install.cmake
│   │   │   ├── CTestConfiguration.ini
│   │   │   ├── CTestCustom.cmake
│   │   │   ├── CTestTestfile.cmake
│   │   │   ├── gtest
│   │   │   │   ├── CMakeFiles
│   │   │   │   │   ├── CMakeDirectoryInformation.cmake
│   │   │   │   │   └── progress.marks
│   │   │   │   ├── cmake_install.cmake
│   │   │   │   ├── CTestTestfile.cmake
│   │   │   │   ├── googlemock
│   │   │   │   │   ├── CMakeFiles
│   │   │   │   │   │   ├── CMakeDirectoryInformation.cmake
│   │   │   │   │   │   ├── gmock.dir
│   │   │   │   │   │   │   ├── build.make
│   │   │   │   │   │   │   ├── cmake_clean.cmake
│   │   │   │   │   │   │   ├── DependInfo.cmake
│   │   │   │   │   │   │   ├── depend.make
│   │   │   │   │   │   │   ├── flags.make
│   │   │   │   │   │   │   ├── link.txt
│   │   │   │   │   │   │   └── progress.make
│   │   │   │   │   │   ├── gmock_main.dir
│   │   │   │   │   │   │   ├── build.make
│   │   │   │   │   │   │   ├── cmake_clean.cmake
│   │   │   │   │   │   │   ├── DependInfo.cmake
│   │   │   │   │   │   │   ├── depend.make
│   │   │   │   │   │   │   ├── flags.make
│   │   │   │   │   │   │   ├── link.txt
│   │   │   │   │   │   │   └── progress.make
│   │   │   │   │   │   └── progress.marks
│   │   │   │   │   ├── cmake_install.cmake
│   │   │   │   │   ├── CTestTestfile.cmake
│   │   │   │   │   ├── gtest
│   │   │   │   │   │   ├── CMakeFiles
│   │   │   │   │   │   │   ├── CMakeDirectoryInformation.cmake
│   │   │   │   │   │   │   ├── gtest.dir
│   │   │   │   │   │   │   │   ├── build.make
│   │   │   │   │   │   │   │   ├── cmake_clean.cmake
│   │   │   │   │   │   │   │   ├── DependInfo.cmake
│   │   │   │   │   │   │   │   ├── depend.make
│   │   │   │   │   │   │   │   ├── flags.make
│   │   │   │   │   │   │   │   ├── link.txt
│   │   │   │   │   │   │   │   └── progress.make
│   │   │   │   │   │   │   ├── gtest_main.dir
│   │   │   │   │   │   │   │   ├── build.make
│   │   │   │   │   │   │   │   ├── cmake_clean.cmake
│   │   │   │   │   │   │   │   ├── DependInfo.cmake
│   │   │   │   │   │   │   │   ├── depend.make
│   │   │   │   │   │   │   │   ├── flags.make
│   │   │   │   │   │   │   │   ├── link.txt
│   │   │   │   │   │   │   │   └── progress.make
│   │   │   │   │   │   │   └── progress.marks
│   │   │   │   │   │   ├── cmake_install.cmake
│   │   │   │   │   │   ├── CTestTestfile.cmake
│   │   │   │   │   │   └── Makefile
│   │   │   │   │   └── Makefile
│   │   │   │   └── Makefile
│   │   │   └── Makefile
│   │   ├── devel
│   │   │   ├── cmake.lock
│   │   │   ├── env.sh
│   │   │   ├── local_setup.bash
│   │   │   ├── local_setup.sh
│   │   │   ├── local_setup.zsh
│   │   │   ├── setup.bash
│   │   │   ├── setup.sh
│   │   │   ├── _setup_util.py
│   │   │   └── setup.zsh
│   │   ├── mock
│   │   ├── MV-SUA134GC-Group0.config
│   │   ├── package.json
│   │   ├── README.md
│   │   ├── src
│   │   │   ├── app.js
│   │   │   ├── assets
│   │   │   │   ├── minimap.png
│   │   │   │   └── noCamera.png
│   │   │   ├── CMakeLists.txt -> /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
│   │   │   ├── components
│   │   │   │   ├── calibrator.js
│   │   │   │   ├── cameraView.js
│   │   │   │   ├── healthBar
│   │   │   │   │   ├── index.js
│   │   │   │   │   ├── progress.css
│   │   │   │   │   └── progress.js
│   │   │   │   ├── hp.js
│   │   │   │   ├── minimap.js
│   │   │   │   └── pageLoading.js
│   │   │   ├── displayerBackend.js
│   │   │   ├── layouts
│   │   │   │   ├── index.css
│   │   │   │   └── index.js
│   │   │   ├── models
│   │   │   │   ├── configProvider.js
│   │   │   │   └── robotStatus.js
│   │   │   └── pages
│   │   │       ├── calibrate.jsx
│   │   │       └── index.jsx
│   │   ├── tsconfig.json
│   │   └── typings.d.ts
│   ├── game_state_publisher
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── src
│   │       └── game_state_publisher_node.cpp
│   ├── get_depth
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── src
│   │       ├── get_depth_node.cpp
│   │       └── project
│   │           ├── CMakeLists.txt
│   │           ├── main.cpp
│   │           ├── project.cu
│   │           └── project.h
│   ├── get_depth_node性能分析.png
│   ├── getPictures
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── src
│   │       └── getPictures_node.cpp
│   ├── hp_limit_helper
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── hp_limit_helper
│   │   │       └── robot.h
│   │   ├── msg
│   │   │   ├── RobotHP.msg
│   │   │   └── RobotsHP.msg
│   │   ├── package.xml
│   │   └── src
│   │       ├── main.cpp
│   │       └── robot.cpp
│   ├── mv_driver
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   ├── CameraApi.h
│   │   │   ├── CameraDefine.h
│   │   │   ├── CameraStatus.h
│   │   │   ├── MVCamera.h
│   │   │   └── video_saver.h
│   │   ├── lib
│   │   │   ├── arm
│   │   │   │   └── libMVSDK.so
│   │   │   ├── arm64
│   │   │   │   └── libMVSDK.so
│   │   │   ├── x64
│   │   │   │   └── libMVSDK.so
│   │   │   └── x86
│   │   │       └── libMVSDK.so
│   │   ├── package.xml
│   │   └── src
│   │       ├── MVCamera.cpp
│   │       └── MVCamera_node.cpp
│   ├── poseEstimation
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── src
│   │       └── poseEstimation_node.cpp
│   ├── radar_msgs
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   └── radar.launch
│   │   ├── msg
│   │   │   ├── dist_point.msg
│   │   │   ├── dist_points.msg
│   │   │   ├── game_state.msg
│   │   │   ├── point.msg
│   │   │   ├── points.msg
│   │   │   ├── referee_warning.msg
│   │   │   ├── relative_coordinate.msg
│   │   │   ├── small_map.msg
│   │   │   ├── supply_projectile_action.msg
│   │   │   ├── world_point.msg
│   │   │   ├── yolo_point.msg
│   │   │   └── yolo_points.msg
│   │   ├── package.xml
│   │   └── yaml
│   │       └── radar.yaml
│   ├── README.md
│   ├── realsense_ros_driver
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── realsense_ros_driver
│   │   ├── package.xml
│   │   └── src
│   │       └── realsense_ros_driver_node.cpp
│   ├── serial_port
│   │   ├── cfg
│   │   │   └── radar_station.yaml
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   ├── CRC8_CRC16.h
│   │   │   └── CRC.h
│   │   ├── package.xml
│   │   └── src
│   │       └── serial_port_node.cpp
│   ├── sjtu_battlefield.jpg
│   ├── small_map
│   │   ├── CMakeLists.txt
│   │   ├── launch
│   │   │   └── small_map.launch
│   │   ├── package.xml
│   │   └── src
│   │       ├── 增益区.png
│   │       ├── battlefield.png
│   │       ├── blue_minimap.png
│   │       ├── red_minimap.png
│   │       ├── small_map.cpp
│   │       └── warn_regions.png
│   ├── video_pub
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   ├── gpu_timer.h
│   │   │   ├── preprocess.h
│   │   │   └── preprocess_kernel.cuh
│   │   ├── package.xml
│   │   └── src
│   │       ├── preprocess.cpp
│   │       ├── preprocess_kernel.cu
│   │       └── video_pub.cpp
│   ├── yolo
│   │   ├── cmake-build-debug
│   │   │   ├── CMakeCache.txt
│   │   │   └── CMakeFiles
│   │   │       ├── 3.21.1
│   │   │       │   ├── CMakeCCompiler.cmake
│   │   │       │   ├── CMakeCXXCompiler.cmake
│   │   │       │   ├── CMakeDetermineCompilerABI_C.bin
│   │   │       │   ├── CMakeDetermineCompilerABI_CXX.bin
│   │   │       │   ├── CMakeSystem.cmake
│   │   │       │   ├── CompilerIdC
│   │   │       │   │   ├── a.out
│   │   │       │   │   └── CMakeCCompilerId.c
│   │   │       │   └── CompilerIdCXX
│   │   │       │       ├── a.out
│   │   │       │       └── CMakeCXXCompilerId.cpp
│   │   │       ├── clion-environment.txt
│   │   │       ├── clion-log.txt
│   │   │       ├── cmake.check_cache
│   │   │       └── CMakeOutput.log
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── yolo
│   │   │       ├── calibrator.h
│   │   │       ├── common.hpp
│   │   │       ├── cuda_utils.h
│   │   │       ├── logging.h
│   │   │       ├── macros.h
│   │   │       ├── preprocess.h
│   │   │       ├── utils.h
│   │   │       └── yololayer.h
│   │   ├── package.xml
│   │   └── src
│   │       ├── calibrator.cpp
│   │       ├── preprocess.cu
│   │       ├── yololayer.cu
│   │       └── yolov5.cpp
│   └── yolo_with_two_layers
│       ├── 1.jpg
│       ├── CMakeLists.txt
│       ├── include
│       │   └── yolo_with_two_layers
│       │       ├── calibrator.h
│       │       ├── common.hpp
│       │       ├── cuda_utils.h
│       │       ├── logging.h
│       │       ├── macros.h
│       │       ├── preprocess.h
│       │       ├── utils.h
│       │       └── yololayer.h
│       ├── package.xml
│       ├── src
│       │   ├── calibrator.cpp
│       │   ├── preprocess.cu
│       │   ├── yololayer.cu
│       │   └── yolov5.cpp
│       ├── yolov5s_car.engine
│       └── yolov5s_number.engine
└── tree.md

## 项目负责人

## 开源协议

![image](https://github.com/DoveJH/readme_source/blob/main/2020radar_station/%E9%9B%B7%E8%BE%BE%E7%BB%84%E6%A0%87(1).png)
