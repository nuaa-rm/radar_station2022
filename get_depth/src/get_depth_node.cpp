//
// Created by dovejh on 2022/3/19.
//
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace std;
using namespace cv;

int imgRows = 720, imgCols = 1280;
Mat camera_matrix = Mat_<double>(3, 3);
Mat distortion_coefficient = Mat_<double>(5, 1);
Mat uni_matrix = Mat_<double>(3, 4);

void depthShow(Mat& depths)
{
    static Mat depthsGray(imgRows, imgCols, CV_8U);
    double min = 0, max = 10000;
    for(int i = 0; i < imgRows; i++)
    {
        for(int j = 0; j < imgCols; j++)
        {
            if(min > depths.at<double>(i, j))
            {
                min = depths.at<double>(i, j);
            }
            else if(max < depths.at<double>(i, j))
            {
                max = depths.at<double>(i, j);
            }
        }
    }
    for(int i = 0; i < imgRows; i++)
    {
        for(int j = 0; j < imgCols; j++)
        {
            depthsGray.at<uchar>(i, j) = depths.at<double>(i, j) / (max - min) * 255;
        }
    }
    imshow("depthGray", depthsGray);
    waitKey(30);
}

long long time_state = 0;
void getTheoreticalUV(double x, double y, double z) //图像必须提前矫正
{
    static Mat depths = Mat_<double>(imgRows, imgCols);
    double matrix3[4][1] = {x, y, z, 1};

    // transform into the opencv matrix*/
    static Mat matrixIn(camera_matrix);
    static Mat matrixOut(uni_matrix);
    Mat coordinate(4, 1, CV_64F, matrix3);

    // calculate the result of u and v
    Mat result = camera_matrix*matrixOut*coordinate;
    float u = result.at<double>(0, 0);
    float v = result.at<double>(1, 0);
    float depth = result.at<double>(2, 0);
    if(u <= imgCols && u >= 0 && v <= imgRows && v >= 0)
    {
        depths.at<double>(u, v) = x;
    }
}


void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("realtime pcl"));
    //pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;    //原始的点云的数据格式
    //pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    // 转化为PCL中的点云的数据格式
    //pcl_conversions::toPCL(*input, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1;
    cloud1.reset (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input, *cloud1);


    viewer->removeAllPointClouds();  // 移除当前所有点云
    viewer->addPointCloud(cloud1, "realtime pcl");
    viewer->updatePointCloud(cloud1, "realtime pcl");
    viewer->spinOnce(0.001);

    std::cout << cloud1->points.size() << std::endl;
    viewer->addPointCloud(cloud1, "realtime pcl");


    //摁“Q”键结束
    //viewer->spin();
    //std::cout << "done." << std::endl;

    //while (!viewer->wasStopped())
    //{
        //viewer->spinOnce(100);   //100??
        //boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    //}
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_depth_node");
    ros::NodeHandle n;

    ros::param::get("/image_width", imgCols);
    ros::param::get("/image_height", imgRows);

    ros::param::get("/camera_matrix/zerozero", camera_matrix.at<double>(0, 0));
    ros::param::get("/camera_matrix/zerotwo", camera_matrix.at<double>(0, 2));
    ros::param::get("/camera_matrix/oneone", camera_matrix.at<double>(1, 1));
    ros::param::get("/camera_matrix/onetwo", camera_matrix.at<double>(1, 2));
    ros::param::get("/camera_matrix/twotwo", camera_matrix.at<double>(2, 2));
    camera_matrix.at<double>(0, 1) = 0;
    camera_matrix.at<double>(1, 0) = 0;
    camera_matrix.at<double>(2, 0) = 0;
    camera_matrix.at<double>(2, 1) = 0;
    cout << "Camera matrix load done!" << camera_matrix <<endl;

    /*ros::param::get("/distortion_coefficient/zero", distortion_coefficient.at<float>(0, 0));
    ros::param::get("/distortion_coefficient/one", distortion_coefficient.at<float>(0, 1));
    ros::param::get("/distortion_coefficient/two", distortion_coefficient.at<float>(0, 2));
    ros::param::get("/distortion_coefficient/three", distortion_coefficient.at<float>(0, 3));
    ros::param::get("/distortion_coefficient/four", distortion_coefficient.at<float>(0, 4));*/
    distortion_coefficient.at<double>(0, 0) = 0;
    distortion_coefficient.at<double>(0, 1) = 0;
    distortion_coefficient.at<double>(0, 2) = 0;
    distortion_coefficient.at<double>(0, 3) = 0;
    distortion_coefficient.at<double>(0, 4) = 0;
    cout << "Distortion coefficient load done!" <<distortion_coefficient << endl;

    ros::param::get("/uni_matrix/zerozero", uni_matrix.at<double>(0, 0));
    ros::param::get("/uni_matrix/zeroone", uni_matrix.at<double>(0, 1));
    ros::param::get("/uni_matrix/zerotwo", uni_matrix.at<double>(0, 2));
    ros::param::get("/uni_matrix/zerothree", uni_matrix.at<double>(0, 3));
    ros::param::get("/uni_matrix/onezero", uni_matrix.at<double>(1, 0));
    ros::param::get("/uni_matrix/oneone", uni_matrix.at<double>(1, 1));
    ros::param::get("/uni_matrix/onetwo", uni_matrix.at<double>(1, 2));
    ros::param::get("/uni_matrix/onethree", uni_matrix.at<double>(1, 3));
    ros::param::get("/uni_matrix/twozero", uni_matrix.at<double>(2, 0));
    ros::param::get("/uni_matrix/twoone", uni_matrix.at<double>(2, 1));
    ros::param::get("/uni_matrix/twotwo", uni_matrix.at<double>(2, 2));
    ros::param::get("/uni_matrix/twothree", uni_matrix.at<double>(2, 3));
    cout << "Uni matrix load done!" <<uni_matrix << endl;

    ros::Subscriber sub;
    sub = n.subscribe ("/livox/lidar", 1, &pointCloudCallback);

    ros::spin();
    return 0;
}
