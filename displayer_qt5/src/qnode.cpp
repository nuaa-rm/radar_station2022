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

bool QNode::init() {
	ros::init(init_argc,init_argv,"displayer_qt5");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    image_transport::ImageTransport it(n);
    image_sub = it.subscribe("/sensor_far/image_raw",1,&QNode::imgShowCallback,this);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"displayer_qt5");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    image_transport::ImageTransport it(n);
    image_sub = it.subscribe("/sensor_far/image_raw",1,&QNode::imgShowCallback,this);
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

}  // namespace displayer_qt5
