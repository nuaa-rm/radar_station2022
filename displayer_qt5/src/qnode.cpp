/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/displayer_qt5/qnode.hpp"
#include "sensor_msgs/image_encodings.h"
#include "displayer_qt5/qlabel_with_mouse_event.h"
#include <radar_msgs/dist_points.h>
#include <radar_msgs/dist_point.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace displayer_qt5 {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
    {}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

void QNode::imgShowCallback(const sensor_msgs::ImageConstPtr &msg)
{
    if(!if_is_celibrating)
    {
        try
        {
          cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
          if(!cv_ptr->image.empty())
          {
              img = cv_ptr->image;
              cv::resize(img, img, cv::Size(showMainWindowWidth, showMainWindowHeight));
              image = QImage(img.data,img.cols,img.rows,img.step[0],QImage::Format_RGB888);//change  to QImage format
          }
          Q_EMIT loggingCamera();
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
}

void QNode::imgSensorFarCallback(const sensor_msgs::ImageConstPtr &msg)
{
    if(cameraCelibrating == sensorFarImgRaw && if_is_celibrating)
    {
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
            if(!cv_ptr->image.empty())
            {
                imgSensorFar = cv_ptr->image;
                cv::resize(imgSensorFar, imgSensorFar, cv::Size(calibrateMainWindowWidth, calibrateMainWindowHeight));
                imageCalibrateMainWindow = QImage(imgSensorFar.data,imgSensorFar.cols,imgSensorFar.rows,imgSensorFar.step[0],QImage::Format_RGB888);//change  to QImage format
                Q_EMIT loggingCameraCalibrateMainWindow();
                Q_EMIT loggingCameraCalibrateSecondWindow();
            }
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
}

void QNode::imgSensorCloseCallback(const sensor_msgs::ImageConstPtr &msg)
{
    if(cameraCelibrating == sensorCloseImgRaw && if_is_celibrating)
    {
        try
        {
          cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
          if(!cv_ptr->image.empty())
          {
              imgSensorClose = cv_ptr->image;
              cv::resize(imgSensorClose, imgSensorClose, cv::Size(calibrateMainWindowWidth, calibrateMainWindowHeight));
              imageCalibrateMainWindow = QImage(imgSensorClose.data,imgSensorClose.cols,imgSensorClose.rows,imgSensorClose.step[0],QImage::Format_RGB888);//change  to QImage format
              Q_EMIT loggingCameraCalibrateMainWindow();
              Q_EMIT loggingCameraCalibrateSecondWindow();
          }

        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
}

void QNode::pubCelibrateResult()
{
    radar_msgs::dist_point one_point_msg;
    radar_msgs::dist_points points_msg;
    if(cameraCelibrating == sensorFarImgRaw)
    {
        for(int i = 0; i < 4; i++)
        {
            one_point_msg.x = sensor_far_points[i].x() * 1.0 / calibrateMainWindowWidth;
            one_point_msg.y = sensor_far_points[i].y() * 1.0 / calibrateMainWindowHeight;
            points_msg.data.push_back(one_point_msg);
        }
        calibration_pub_sensor_far.publish(points_msg);
    }
    else if (cameraCelibrating == sensorCloseImgRaw)
    {
        for(int i = 0; i < 4; i++)
        {
            one_point_msg.x = sensor_close_points[i].x() * 1.0 / calibrateMainWindowWidth;
            one_point_msg.y = sensor_close_points[i].y() * 1.0 / calibrateMainWindowHeight;
            points_msg.data.push_back(one_point_msg);
        }
        calibration_pub_sensor_close.publish(points_msg);
    }
}

bool QNode::init()
{
	ros::init(init_argc,init_argv,"displayer_qt5");
    if ( ! ros::master::check() )
    {
		return false;
	}
    loadParams();
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    image_transport::ImageTransport it(n);
    image_sub = it.subscribe(realsenseImgRaw.toStdString(),1,&QNode::imgShowCallback,this);
    image_sub_sensor_far = it.subscribe(sensorFarImgRaw.toStdString(),1,&QNode::imgSensorFarCallback,this);
    image_sub_sensor_close = it.subscribe(sensorCloseImgRaw.toStdString(),1,&QNode::imgSensorCloseCallback,this);
    calibration_pub_sensor_far = n.advertise <radar_msgs::dist_points>(calibrationTopicSensorFar.toStdString(), 1);
    calibration_pub_sensor_close = n.advertise <radar_msgs::dist_points>(calibrationTopicSensorClose.toStdString(), 1);
	start();
	return true;
}


void QNode::run()
{

    log(Info,"Running!");
    ros::spin();
    std::cout << "Ros shutdown" << std::endl;
    Q_EMIT rosShutdown();
}


void QNode::log( const LogLevel &level, const std::string &msg) {
    logInformation = new log_information;
    logInformation->level = level;
    std::stringstream logging_model_msg;
    switch ( level ) {
        case(Debug) : {
                ROS_DEBUG_STREAM(msg);
                logging_model_msg << msg;
                break;
        }
        case(Info) : {
                ROS_INFO_STREAM(msg);
                logging_model_msg << msg;
                break;
        }
        case(Warn) : {
                ROS_WARN_STREAM(msg);
                logging_model_msg << msg;
                break;
        }
        case(Error) : {
                ROS_ERROR_STREAM(msg);
                logging_model_msg << msg;
                break;
        }
        case(Fatal) : {
                ROS_FATAL_STREAM(msg);
                logging_model_msg << msg;
                break;
        }
    }
    logInformation->qstring = (QString(logging_model_msg.str().c_str()));
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::loadParams()
{
    std::string str;
    ros::param::get("/camera/list/farCam/topic", str);
    sensorFarImgRaw = QString(str.c_str());
    ros::param::get("/camera/list/closeCam/topic", str);
    sensorCloseImgRaw = QString(str.c_str());

    ros::param::get("/calibrate/rate", calibrateRate);

    ros::param::get("/camera/list/realsense/topic", str);
    realsenseImgRaw = QString(str.c_str());

    ros::param::get("/camera/list/farCam/calibrationTopic", str);
    calibrationTopicSensorFar = QString(str.c_str());
    ros::param::get("/camera/list/closeCam/calibrationTopic", str);
    calibrationTopicSensorClose = QString(str.c_str());

    battle_color = "blue";
    ros::param::get("/battle_color", battle_color);

    smallMapWidth = 360;
    smallMapHeight = 672;

    std::string ad(PROJECT_PATH);
    if(battle_color == std::string("red"))
    {
        ad += "/resources/images/blue_minimap.png";
    }
    else if(battle_color == std::string("blue"))
    {
        ad += "/resources/images/red_minimap.png";
    }
    imgSmallMap = cv::imread(ad);

    cv::resize(imgSmallMap, imgSmallMap, cv::Size(smallMapWidth, smallMapHeight));
    imageSmallMap = QImage(imgSmallMap.data,imgSmallMap.cols,imgSmallMap.rows,imgSmallMap.step[0],QImage::Format_RGB888);

    logoHeight = 448;
    logoWidth = 222;
    ad = std::string(PROJECT_PATH);
    ad += "/resources/images/radar_logo.jpg";
    imgLogo = cv::imread(ad);
    cv::resize(imgLogo, imgLogo, cv::Size(logoWidth, logoHeight));
    imageLogo = QImage(imgLogo.data,imgLogo.cols,imgLogo.rows,imgLogo.step[0],QImage::Format_RGB888);

    calibrateMainWindowWidth = 1256;
    calibrateMainWindowHeight = 1005;
    calibrateSecondWindowWidth = 618;
    calibrateSecondWindowHeight = 618;

    showMainWindowWidth = 1280;
    showMainWindowHeight = 720;
    showSecondWindowWidth = 346;
    showSecondWindowHeight = 277;

    img = cv::Mat(showMainWindowHeight, showMainWindowWidth, CV_8UC3, cv::Scalar(255, 255, 255));
    imgSensorFar = cv::Mat(calibrateMainWindowHeight, calibrateMainWindowWidth, CV_8UC3, cv::Scalar(255, 255, 255));
    imgSensorClose = cv::Mat(calibrateMainWindowHeight, calibrateMainWindowWidth, CV_8UC3, cv::Scalar(255, 255, 255));

    ad = std::string(PROJECT_PATH);
    ad += "/resources/images/icon.png";
    imgShowSecondWindow = cv::imread(ad);
    cv::resize(imgShowSecondWindow, imgShowSecondWindow, cv::Size(showSecondWindowWidth, showSecondWindowHeight));
    imageShowSecondWindow = QImage(imgShowSecondWindow.data,imgShowSecondWindow.cols,imgShowSecondWindow.rows,imgShowSecondWindow.step[0],QImage::Format_RGB888);

    if_is_celibrating = false;

    float x, y;
    QPoint point;
    ros::param::get("/camera/list/farCam/calibrationDefault/point1/x", x);
    ros::param::get("/camera/list/farCam/calibrationDefault/point1/y", y);
    point = QPoint(x * calibrateMainWindowWidth, y * calibrateMainWindowHeight);
    sensor_far_points[0] = point;
    ros::param::get("/camera/list/farCam/calibrationDefault/point2/x", x);
    ros::param::get("/camera/list/farCam/calibrationDefault/point2/y", y);
    point = QPoint(x * calibrateMainWindowWidth, y * calibrateMainWindowHeight);
    sensor_far_points[1] = point;
    ros::param::get("/camera/list/farCam/calibrationDefault/point3/x", x);
    ros::param::get("/camera/list/farCam/calibrationDefault/point3/y", y);
    point = QPoint(x * calibrateMainWindowWidth, y * calibrateMainWindowHeight);
    sensor_far_points[2] = point;
    ros::param::get("/camera/list/farCam/calibrationDefault/point4/x", x);
    ros::param::get("/camera/list/farCam/calibrationDefault/point4/y", y);
    point = QPoint(x * calibrateMainWindowWidth, y * calibrateMainWindowHeight);
    sensor_far_points[3] = point;
    ros::param::get("/camera/list/closeCam/calibrationDefault/point1/x", x);
    ros::param::get("/camera/list/closeCam/calibrationDefault/point1/y", y);
    point = QPoint(x * calibrateMainWindowWidth, y * calibrateMainWindowHeight);
    sensor_close_points[0] = point;
    ros::param::get("/camera/list/closeCam/calibrationDefault/point2/x", x);
    ros::param::get("/camera/list/closeCam/calibrationDefault/point2/y", y);
    point = QPoint(x * calibrateMainWindowWidth, y * calibrateMainWindowHeight);
    sensor_close_points[1] = point;
    ros::param::get("/camera/list/closeCam/calibrationDefault/point3/x", x);
    ros::param::get("/camera/list/closeCam/calibrationDefault/point3/y", y);
    point = QPoint(x * calibrateMainWindowWidth, y * calibrateMainWindowHeight);
    sensor_close_points[2] = point;
    ros::param::get("/camera/list/closeCam/calibrationDefault/point4/x", x);
    ros::param::get("/camera/list/closeCam/calibrationDefault/point4/y", y);
    point = QPoint(x * calibrateMainWindowWidth, y * calibrateMainWindowHeight);
    sensor_close_points[3] = point;
}

}  // namespace displayer_qt5
