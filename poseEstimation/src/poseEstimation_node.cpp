//
// Created by dovejh on 2022/1/11.
//
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#define POINT_NUMBER 4

using namespace cv;
using namespace std;

Mat template_one, template_two, template_three, template_four;
Mat camera_matrix = Mat_<float>(3, 3);
Mat distortion_coefficient = Mat_<float>(5, 1);
vector<Point3f> PointSetsForTest;
vector<Point3f> PointSets;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

    //模板匹配
    Mat result;
    double maxVal, minVal;
    Point minLoc, maxLoc;
    static vector<Point2f>imgPoints;

    matchTemplate(img, template_one, result, TM_CCOEFF_NORMED);//模板匹配
    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
    imgPoints.push_back(maxLoc);
    rectangle(img, cv::Rect(maxLoc.x, maxLoc.y, template_one.cols, template_one.rows), Scalar(0, 0, 255), 2);

    matchTemplate(img, template_two, result, TM_CCOEFF_NORMED);//模板匹配
    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
    imgPoints.push_back(maxLoc);
    rectangle(img, cv::Rect(maxLoc.x, maxLoc.y, template_two.cols, template_two.rows), Scalar(0, 0, 255), 2);

    matchTemplate(img, template_three, result, TM_CCOEFF_NORMED);//模板匹配
    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
    imgPoints.push_back(maxLoc);
    rectangle(img, cv::Rect(maxLoc.x, maxLoc.y, template_three.cols, template_three.rows), Scalar(0, 0, 255), 2);

    matchTemplate(img, template_four, result, TM_CCOEFF_NORMED);//模板匹配
    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
    imgPoints.push_back(maxLoc);
    rectangle(img, cv::Rect(maxLoc.x, maxLoc.y, template_four.cols, template_four.rows), Scalar(0, 0, 255), 2);

    Mat rvec, tvec; //旋转向量和平移向量
    solvePnPRansac(PointSets, imgPoints, camera_matrix, distortion_coefficient, rvec, tvec, SOLVEPNP_EPNP);
    Mat RRansac;
    Rodrigues(rvec, RRansac);
    cout << "旋转向量转换成旋转矩阵：" << endl << RRansac << endl;
    cout << "平移向量："  <<endl << tvec << endl;

    //这段代码的作用是选取一个三维空间坐标进行投影，验证pnp效果。
    vector<Point2f> imagePointsForTest;
    projectPoints(PointSetsForTest, rvec, tvec, camera_matrix, distortion_coefficient, imagePointsForTest);
    circle(img, imagePointsForTest[0], 5, Scalar(0, 0, 255), 2);
    circle(img, imagePointsForTest[1], 5, Scalar(0, 0, 255), 2);
    circle(img, imagePointsForTest[2], 5, Scalar(0, 0, 255), 2);

    imshow("realsense", img);
    int k = waitKey(30);
    if(k == 's' && !img.empty())
    {
        cv::imwrite("/home/dovejh/project/rader_station/src/rader_station2022/poseEstimation/templates/0.jpg", img);
        ROS_INFO("Get picture!");
    }
    imgPoints.clear();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "poseEstimation_node");

    //读取模板文件
    template_one = imread("/home/dovejh/project/rader_station/src/rader_station2022/poseEstimation/templates/1.jpg");
    template_two = imread("/home/dovejh/project/rader_station/src/rader_station2022/poseEstimation/templates/2.jpg");
    template_three = imread("/home/dovejh/project/rader_station/src/rader_station2022/poseEstimation/templates/3.jpg");
    template_four = imread("/home/dovejh/project/rader_station/src/rader_station2022/poseEstimation/templates/4.jpg");
    cout << "Template load done!" << endl;

    ros::param::get("/camera_matrix/zerozero", camera_matrix.at<float>(0, 0));
    ros::param::get("/camera_matrix/zerotwo", camera_matrix.at<float>(0, 2));
    ros::param::get("/camera_matrix/oneone", camera_matrix.at<float>(1, 1));
    ros::param::get("/camera_matrix/onetwo", camera_matrix.at<float>(1, 2));
    ros::param::get("/camera_matrix/twotwo", camera_matrix.at<float>(2, 2));
    camera_matrix.at<float>(0, 1) = 0;
    camera_matrix.at<float>(1, 0) = 0;
    camera_matrix.at<float>(2, 0) = 0;
    camera_matrix.at<float>(2, 1) = 0;

    ros::param::get("/distortion_coefficient/zero", distortion_coefficient.at<float>(0, 0));
    ros::param::get("/distortion_coefficient/one", distortion_coefficient.at<float>(0, 1));
    ros::param::get("/distortion_coefficient/two", distortion_coefficient.at<float>(0, 2));
    ros::param::get("/distortion_coefficient/three", distortion_coefficient.at<float>(0, 3));
    ros::param::get("/distortion_coefficient/four", distortion_coefficient.at<float>(0, 4));
    /*for(int i = 0; i < 5; i++)
    {
        distortion_coefficient.at<float>(i, 0) = 0;
    }*/
    cout << camera_matrix <<endl;
    cout << distortion_coefficient << endl;

    Point3f objectPoint;
    ros::param::get("/object_points/one/x", objectPoint.x);
    ros::param::get("/object_points/one/y", objectPoint.y);
    ros::param::get("/object_points/one/z", objectPoint.z);
    PointSets.push_back(objectPoint);
    ros::param::get("/object_points/two/x", objectPoint.x);
    ros::param::get("/object_points/two/y", objectPoint.y);
    ros::param::get("/object_points/two/z", objectPoint.z);
    PointSets.push_back(objectPoint);
    ros::param::get("/object_points/three/x", objectPoint.x);
    ros::param::get("/object_points/three/y", objectPoint.y);
    ros::param::get("/object_points/three/z", objectPoint.z);
    PointSets.push_back(objectPoint);
    ros::param::get("/object_points/four/x", objectPoint.x);
    ros::param::get("/object_points/four/y", objectPoint.y);
    ros::param::get("/object_points/four/z", objectPoint.z);
    PointSets.push_back(objectPoint);
    cout << "Object points load done!" << endl;

    Point3f pointForTest;
    pointForTest.x = 140;
    pointForTest.y = 58;
    pointForTest.z = 0;
    PointSetsForTest.push_back(pointForTest);
    pointForTest.x = 0;
    pointForTest.y = 0;
    pointForTest.z = 0;
    PointSetsForTest.push_back(pointForTest);
    pointForTest.x = 140;
    pointForTest.y = 58;
    pointForTest.z = 220;
    PointSetsForTest.push_back(pointForTest);
    cout << "Test points load done!" << endl;

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/camera/color/image_raw", 1, &imageCallback);
    ros::spin();
    return 0;
}