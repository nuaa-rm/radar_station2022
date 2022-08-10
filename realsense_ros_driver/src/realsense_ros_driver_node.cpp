#include "ros/ros.h"
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Image.h>
#include <iostream>
#include<string>
#include <math.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/hpp/rs_types.hpp>
#include <librealsense2/hpp/rs_sensor.hpp>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

// 相机内参
const double camera_factor = 1000;
const double camera_cx = 311.704;
const double camera_cy = 245.82;
const double camera_fx = 474.055;
const double camera_fy = 474.055;

ros::Publisher img_pub;


void get_img(ros::NodeHandle nh)
{
    rs2::pipeline pipe;     //Contruct a pipeline which abstracts the device
    rs2::config cfg;    //Create a configuration for configuring the pipeline with a non default profile
//
//    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
//
//    rs2::pipeline_profile selection = pipe.start(cfg);
//    rs2_stream align_to = RS2_STREAM_COLOR;
//    rs2::align align(align_to);

    while(ros::ok())
    {
//        rs2::frameset frames;
//        frames = pipe.wait_for_frames();
//
//        auto processed = align.process(frames);
//
//        //Get each frame
//        auto color_frame = processed.get_color_frame();
//
//        //create cv::Mat from rs2::frame
//        Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
//
//        sensor_msgs::ImagePtr color_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color).toImageMsg();
//        color_msg->header.stamp = ros::Time::now();
//        img_pub.publish(color_msg);
    }
}

int main (int argc, char** argv)
{
    //初始化节点
    ros::init(argc, argv, "realsense_ros_driver_node");
    //声明节点句柄
    ros::NodeHandle nh;
    img_pub=nh.advertise<sensor_msgs::Image>("/camera/color/image_raw",10);
    ros::Rate loop_rate(30);

    rs2::pipeline pipe;     //Contruct a pipeline which abstracts the device
    rs2::config cfg;    //Create a configuration for configuring the pipeline with a non default profile

    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    rs2::pipeline_profile selection = pipe.start(cfg);
    rs2_stream align_to = RS2_STREAM_COLOR;
    rs2::align align(align_to);

    while(ros::ok())
    {
        rs2::frameset frames;
        frames = pipe.wait_for_frames();

        auto processed = align.process(frames);

        //Get each frame
        auto color_frame = processed.get_color_frame();

        //create cv::Mat from rs2::frame
        Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

        sensor_msgs::ImagePtr color_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color).toImageMsg();
        color_msg->header.stamp = ros::Time::now();
        img_pub.publish(color_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    nh.shutdown();
}