# RadarDisplaywer
基于react的RoboMaster雷达站前端项目  
南京航空航天大学长空御风战队开源
## 主要技术框架
前端: react, umijs, dvajs, ant design, fabricjs  
后端: flask, rospy  
GUI: pywebview
## 项目特点
* 前后端分离设计，前后端通过websocket进行通信（不可用时降级为长轮询）
* 使用pywebview提供桌面GUI，同时保留浏览器接口，可通过手机等设备远程监看
* 配置化实现多摄像头显示，支持通过topic在前端绘图
* 支持前端绘图通过topic发送，用于相机外参标定等场景
* 松耦合设计，提供后端接口模板类，ros接口基于该模板类实现，可简单脱离ros环境使用
## 构建
要构建本工程需要安装nodejs环境，本例采用yarn做包管理
```bash
git clone https://github.com/bismarckkk/radar_displayer
cd radar_displayer/displayer_web
yarn
yarn build
```
构建完成的前端工程将自动部署至同目录下的displayer_app工程
## 部署
1. 从release下载displayer_app并解压至工作空间，或将本工程clone至工作空间
2. 在displayer_app目录中使用requirements.txt完成依赖包安装
3. 参照示例编辑`displayer_app/cfg/RadarDisplayerRos.yaml`，该配置文件通过ros param读取，
4. 亦可使用其他方式加载配置至ros，
5. 但displayer仅在启动时从ros读取配置
6. 使用catkin_make编译包并生成消息文件
7. `roslaunch displayer_app displayer.launch`
8. 可通过配置文件开关本地显示，外部浏览器可通过终端提示的链接接入监看
## 移植
注：此处移植至移植至除ros1以外的其他环境
参考示例：displayer_app/RosNode.py
从base.py继承BaseNode，BasePublisher，BaseSubscriber，BaseImageSubscriber类并实现，
baseNode为后端接口维护线程，将在完成gevent patch以后被导入，可在该类初始化时完成所有自定义初始化操作  
修改displayer_app/config.py，目前为从rosparam读取配置文件，可移植至其他方案  
然后修改所有的引用路径，完成移植
## 特别注意
后端中大量运用python3，几乎不可能于python2环境运行，如果运行不起来请务必检查使用的python版本  
考虑到很多战队（包括我们自己）仍处于ros melodic版本，很多包对python3存在兼容性问题，
我们已经在requirements.txt中加入部分ros包，除cv_bridge外已经基本支持python3使用，
而cv_bridge需要自行重新编译（参考文章），为了降低使用难度，我们在代码中编写了
部分编码的sensor_msgs/Image消息解析（该功能默认开启），但仅支持bgr8和rgb8编码，
如需使用其他编码进行传输请自行编译python3适用的cv_bridge并在配置文件中开启cv_bridge支持