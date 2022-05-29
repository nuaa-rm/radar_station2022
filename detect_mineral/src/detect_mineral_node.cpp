//
// Created by dovejh on 2022/5/29.
//
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

void imgCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_mineral_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/camera/color/image_raw", 1, &imgCallback);
    ros::spin();
}