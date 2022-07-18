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
          img = cv_ptr->image;
          cv::resize(img, img, cv::Size(1536, 864));
          image = QImage(img.data,img.cols,img.rows,img.step[0],QImage::Format_RGB888);//change  to QImage format
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
          imgSensorFar = cv_ptr->image;
          cv::resize(imgSensorFar, imgSensorFar, cv::Size(calibrateMainWindowWidth, calibrateMainWindowHeight));
          imageCalibrateMainWindow = QImage(imgSensorFar.data,imgSensorFar.cols,imgSensorFar.rows,imgSensorFar.step[0],QImage::Format_RGB888);//change  to QImage format
          Q_EMIT loggingCameraCalibrateMainWindow();

          cv::Rect r;
          r.width = calibrateSecondWindowWidth / calibrateRate;
          r.height = calibrateSecondWindowHeight / calibrateRate;
          int halfWidth = (calibrateSecondWindowWidth * 0.5) / calibrateRate;
          int halfHeight = (calibrateSecondWindowHeight * 0.5) / calibrateRate;
          if(mouseLoaction.x() > calibrateMainWindowWidth)
          {
              mouseLoaction.setX(calibrateMainWindowWidth);
          }
          if(mouseLoaction.y() > calibrateMainWindowHeight)
          {
              mouseLoaction.setY(calibrateMainWindowHeight);
          }
          cv::Mat m;
          if(mouseLoaction.x() - halfWidth < 0)
          {
              r.x = 0;
          }
          else if((mouseLoaction.x() + halfWidth) > (calibrateMainWindowWidth))
          {
              r.x = calibrateMainWindowWidth - calibrateSecondWindowWidth / calibrateRate;
          }
          else
          {
              r.x = mouseLoaction.x() - halfWidth;
          }

          if(mouseLoaction.y() - halfHeight < 0)
          {
              r.y = 0;
          }
          else if((mouseLoaction.y() + halfHeight) > (calibrateMainWindowHeight))
          {
              r.y = calibrateMainWindowHeight - calibrateSecondWindowHeight / calibrateRate;
          }
          else
          {
              r.y = mouseLoaction.y() - halfHeight;
          }
          if (r.x < 0)
          {
              r.x = 0;
          }
          if (r.y < 0)
          {
              r.y = 0;
          }
          if ((r.x + r.width) > imgSensorFar.cols)
          {
              r.width = imgSensorFar.cols - r.x;
          }
          if ((r.y + r.height) > imgSensorFar.rows)
          {
              r.height = imgSensorFar.rows - r.y;
          }
          imgSensorFar(r).copyTo(m);
          cv::resize(m, m, cv::Size(calibrateSecondWindowWidth, calibrateSecondWindowHeight));
          cv::line(m, cv::Point(0, (mouseLoaction.y() - r.y) * calibrateRate), cv::Point(calibrateSecondWindowWidth, (mouseLoaction.y() - r.y) * calibrateRate), cv::Scalar(255, 255, 255));
          cv::line(m, cv::Point((mouseLoaction.x() - r.x) * calibrateRate, 0), cv::Point((mouseLoaction.x() - r.x) * calibrateRate, calibrateSecondWindowHeight), cv::Scalar(255, 255, 255));
          imageCalibrateSecondWindow = QImage(m.data,m.cols,m.rows,m.step[0],QImage::Format_RGB888);
          Q_EMIT loggingCameraCalibrateSecondWindow();
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
          imgSensorClose = cv_ptr->image;
          cv::resize(imgSensorClose, imgSensorClose, cv::Size(calibrateMainWindowWidth, calibrateMainWindowHeight));
          imageCalibrateMainWindow = QImage(imgSensorClose.data,imgSensorClose.cols,imgSensorClose.rows,imgSensorClose.step[0],QImage::Format_RGB888);//change  to QImage format
          Q_EMIT loggingCameraCalibrateMainWindow();

          cv::Rect r;
          r.width = calibrateSecondWindowWidth / calibrateRate;
          r.height = calibrateSecondWindowHeight / calibrateRate;
          int halfWidth = (calibrateSecondWindowWidth * 0.5) / calibrateRate;
          int halfHeight = (calibrateSecondWindowHeight * 0.5) / calibrateRate;
          if(mouseLoaction.x() > calibrateMainWindowWidth)
          {
              mouseLoaction.setX(calibrateMainWindowWidth);
          }
          if(mouseLoaction.y() > calibrateMainWindowHeight)
          {
              mouseLoaction.setY(calibrateMainWindowHeight);
          }
          cv::Mat m;
          if(mouseLoaction.x() - halfWidth < 0)
          {
              r.x = 0;
          }
          else if((mouseLoaction.x() + halfWidth) > (calibrateMainWindowWidth))
          {
              r.x = calibrateMainWindowWidth - calibrateSecondWindowWidth / calibrateRate;
          }
          else
          {
              r.x = mouseLoaction.x() - halfWidth;
          }

          if(mouseLoaction.y() - halfHeight < 0)
          {
              r.y = 0;
          }
          else if((mouseLoaction.y() + halfHeight) > (calibrateMainWindowHeight))
          {
              r.y = calibrateMainWindowHeight - calibrateSecondWindowHeight / calibrateRate;
          }
          else
          {
              r.y = mouseLoaction.y() - halfHeight;
          }
          if (r.x < 0)
          {
              r.x = 0;
          }
          if (r.y < 0)
          {
              r.y = 0;
          }
          if ((r.x + r.width) > imgSensorClose.cols)
          {
              r.width = imgSensorClose.cols - r.x;
          }
          if ((r.y + r.height) > imgSensorClose.rows)
          {
              r.height = imgSensorClose.rows - r.y;
          }
          imgSensorClose(r).copyTo(m);
          cv::resize(m, m, cv::Size(calibrateSecondWindowWidth, calibrateSecondWindowHeight));
          cv::line(m, cv::Point(0, (mouseLoaction.y() - r.y) * calibrateRate), cv::Point(calibrateSecondWindowWidth, (mouseLoaction.y() - r.y) * calibrateRate), cv::Scalar(255, 255, 255));
          cv::line(m, cv::Point((mouseLoaction.x() - r.x) * calibrateRate, 0), cv::Point((mouseLoaction.x() - r.x) * calibrateRate, calibrateSecondWindowHeight), cv::Scalar(255, 255, 255));
          imageCalibrateSecondWindow = QImage(m.data,m.cols,m.rows,m.step[0],QImage::Format_RGB888);
          Q_EMIT loggingCameraCalibrateSecondWindow();
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
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
	start();
	return true;
}


void QNode::run() {

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

    calibrateMainWindowWidth = 1280;
    calibrateMainWindowHeight = 1024;
    calibrateSecondWindowWidth = 640;
    calibrateSecondWindowHeight = 640;

    if_is_celibrating = false;

}

}  // namespace displayer_qt5
