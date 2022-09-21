![image](https://github.com/DoveJH/readme_source/blob/main/2020radar_station/%E9%9B%B7%E8%BE%BE%E7%BB%84%E6%A0%87%E6%A8%AA.png)

# 长空御风雷达站

基于工业相机和Livox激光雷达的RoboMaster雷达站项目
由南京航空航天大学长空御风战队开源

## 项目介绍
本项目为南京航空航天大学长空御风战队的RoboMaster2022雷达站开源项目。
![image](https://github.com/DoveJH/readme_source/blob/main/2020radar_station/liucheng.png)

传感器 | 型号
----|----
上相机 *(不参与运算，仅提供视野)* | Realsense D435i
左右相机 | MV-SUA134GC-T 8mm 1:2.0 1/1.8”
激光雷达 | Livox Mid70

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
 - 如果你使用的不是迈德威视工业相机，请启动你的相机节点。并将相机发布的话题修改为/sensor_close/image_raw和/sensor_far/image_raw
 - 根据你的需要修改displayer_qt5/yaml/displayer_qt5.yaml和radar_msgs/yaml/radar.yaml

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
连接相机和激光雷达
    cd [workspace]
    source devel/setup.bash
    roslaunch radar_msgs radar.launch
如果你获得如下效果，即为成功运行。

![image](https://github.com/DoveJH/readme_source/blob/main/2020radar_station/2022-09-17%2012-25-24%20%E7%9A%84%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

## How It Works
![image](https://github.com/DoveJH/readme_source/blob/main/2020radar_station/gongneng.png)

## 项目结构

├── radar_station2022  
│   ├── calibrateCamera 相机标定测试包，已废弃  
│   ├── CMakeLists.txt  
│   ├── dart_detect 一个未完成的飞镖检测节点  
│   ├── detect_mineral 一个未完成的落矿检测节点  
│   ├── displayer_app 旧的UI节点  
│   ├── displayer_qt5 目前使用的UI  
│   │   ├── CMakeLists.txt  
│   │   ├── include  
│   │   │   └── displayer_qt5  
│   │   │       ├── main_window.hpp  
│   │   │       ├── qlabel_with_mouse_event.h  
│   │   │       ├── qlabel_with_painter.h  
│   │   │       └── qnode.hpp  
│   │   ├── mainpage.dox  
│   │   ├── package.xml  
│   │   ├── recorded.avi 每次程序运行生成的文件，可删除  
│   │   ├── resources 用到的资源文件  
│   │   │   ├── images  
│   │   │   │   ├── blue_minimap.png 我方为蓝方时的小地图  
│   │   │   │   ├── CKYF.png  
│   │   │   │   ├── CKYF.svg  
│   │   │   │   ├── Icon.ico  
│   │   │   │   ├── icon.png  
│   │   │   │   ├── radar_logo.png 雷达站LOGO  
│   │   │   │   └── red_minimap.png 我方为红方时的小地图    
│   │   │   ├── images.qrc  
│   │   │   └── qss 进度条样式，用于血条显示    
│   │   │       ├── progressBarBlue.qss  
│   │   │       └── progressBarRed.qss  
│   │   ├── src  
│   │   │   ├── main.cpp 主循环  
│   │   │   ├── main_window.cpp 主窗口类  
│   │   │   ├── qlabel_with_mouse_event.cpp PNP选点实现类  
│   │   │   ├── qlabel_with_painter.cpp 小地图实现类  
│   │   │   └── qnode.cpp ROS节点  
│   │   ├── ui  
│   │   │   └── main_window.ui  
│   │   └── yaml  
│   │       ├── displayer_qt5.yaml UI部分的参数文件  
│   │       └── load_params.launch 用于测试时加载参数的launch  
│   ├── displayer_web 旧的UI节点  
│   ├── game_state_publisher 测试时用于发布一些裁判系统消息的节点  
│   ├── get_depth 深度获取节点，传感器融合的主要工作节点  
│   │   ├── CMakeLists.txt  
│   │   ├── package.xml  
│   │   └── src  
│   │       ├── get_depth_node.cpp  
│   │       └── project 使用CUDA加速投影，已废弃  
│   ├── getPictures 拍图节点，与主程序无关  
│   ├── hp_limit_helper 旧UI用于获取车辆最高血量的节点，已废弃  
│   ├── mv_driver 迈德威视驱动  
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
│   ├── poseEstimation 位姿估计学习节点，与主程序无关  
│   ├── radar_msgs 定义了大部分自定义消息    
│   │   ├── CMakeLists.txt  
│   │   ├── launch  
│   │   │   └── radar.launch 雷达站主程序启动文件  
│   │   ├── msg  
│   │   │   ├── dist_point.msg  
│   │   │   ├── dist_points.msg  
│   │   │   ├── game_state.msg 用于发布从裁判系统获取的游戏状态信息  
│   │   │   ├── point.msg  
│   │   │   ├── points.msg  
│   │   │   ├── referee_warning.msg 从裁判系统读取的判罚信息  
│   │   │   ├── relative_coordinate.msg  
│   │   │   ├── small_map.msg 小地图消息  
│   │   │   ├── supply_projectile_action.msg 从裁判系统读取的补给站消息  
│   │   │   ├── world_point.msg  
│   │   │   ├── yolo_point.msg yolo结果消息  
│   │   │   └── yolo_points.msg  
│   │   ├── package.xml  
│   │   └── yaml  
│   │       └── radar.yaml 雷达的主要参数文件  
│   ├── README.md  
│   ├── realsense_ros_driver realsense驱动  
│   │   ├── CMakeLists.txt  
│   │   ├── include  
│   │   │   └── realsense_ros_driver  
│   │   ├── package.xml  
│   │   └── src  
│   │       └── realsense_ros_driver_node.cpp  
│   ├── serial_port 串口通讯节点  
│   │   ├── cfg  
│   │   │   └── radar_station.yaml  
│   │   ├── CMakeLists.txt  
│   │   ├── include  
│   │   │   ├── CRC8_CRC16.h  
│   │   │   └── CRC.h  
│   │   ├── package.xml  
│   │   └── src  
│   │       └── serial_port_node.cpp  
│   ├── small_map 小地图节点，位置解算的主要节点  
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
│   ├── video_pub 用于从视频发布消息的节点  
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
│   ├── yolo 单模型神经网络，已废弃  
│   └── yolo_with_two_layers 双模型神经网络节点  
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
│       ├── src 大部分代码来自tensorrtx  
│       │   ├── calibrator.cpp  
│       │   ├── preprocess.cu  
│       │   ├── yololayer.cu  
│       │   └── yolov5.cpp  
│       ├── yolov5s_car.engine 用于识别车辆的engine文件  
│       └── yolov5s_number.engine 用于识别数字的engine文件  
└── tree.md

## 主要开发者
[@DoveJH](https://github.com/DoveJH)
[@Christopher](https://github.com/Christopher-bit)
[@bismarckkk](https://github.com/bismarckkk)

## 联系我们
E-mail: 1358446393@qq.com  1134643765@qq.com

## 开源协议

[MIT](LICENSE) © 刘建航

![image](https://github.com/DoveJH/readme_source/blob/main/2020radar_station/%E6%97%A0%E8%83%8C%E6%99%AF2.png)
