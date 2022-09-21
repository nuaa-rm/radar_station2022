# 长空御风雷达站

基于工业相机和Livox激光雷达的RoboMaster雷达站项目
南京航空航天大学长空御风战队开源

[ TOC ]

## 项目介绍
本项目为南京航空航天大学长空御风战队的RoboMaster2022雷达站开源项目。
//机器人功能定义
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
//运行流程图


## 项目负责人

## 开源协议


