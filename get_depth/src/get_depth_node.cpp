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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;
using namespace cv;

int imgRows = 720, imgCols = 1280;
Mat camera_matrix = Mat_<double>(3, 3);
Mat distortion_coefficient = Mat_<double>(5, 1);
Mat uni_matrix = Mat_<double>(3, 4);//相机和雷达的变换矩阵
ros::Publisher depthPub;

void depthShow(Mat& input);//将深度图像归一化成灰度图并发布话题进行展示
void getTheoreticalUV(double x, double y, double z, Mat& output);//得到某一点对应图像中的位置
void cloudFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
void clusterAndSelect(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr& output);//对点云进行聚类,并挑选点数最多的点云作为返回
double getDepthInRect(Rect rect, Mat& depthImg);//得到ROI中点的深度
void removeFlat(pcl::PointCloud<pcl::PointXYZ>::Ptr& input);//去除平面
void projectPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input, Mat& output);//对于每一个点云中的点调用一次getTheoreticalUV函数
double pointCloudShower(pcl::PointCloud<pcl::PointXYZ>::Ptr input);//显示实时点云,放在程序结尾
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);

void depthShow(Mat& input)
{
    Mat depthsGray(imgRows, imgCols, CV_8U);
    double min = 100, max = 0;
    for(int i = 0; i < imgRows; i++)
    {
        for(int j = 0; j < imgCols; j++)
        {
            if(min > input.at<Vec3d>(i, j)[0] && input.at<Vec3d>(i, j)[0] != 0)
            {
                min = input.at<Vec3d>(i, j)[0];
            }
            else if(max < input.at<Vec3d>(i, j)[0])
            {
                max = input.at<Vec3d>(i, j)[0];
            }
        }
    }
    for(int i = 0; i < imgRows; i++)
    {
        for(int j = 0; j < imgCols; j++)
        {
            depthsGray.at<uchar>(i, j) = (input.at<Vec3d>(i, j)[0] / (max - min) * 255.0);
        }
    }
    circle(depthsGray, Point(320, 180), 5, Scalar(255, 255, 255), 1);
    circle(depthsGray, Point(450, 260), 5, Scalar(255, 255, 255), 1);
    rectangle(depthsGray, Point(320, 180), Point(450, 260), Scalar(255, 255, 255), 1);
    sensor_msgs::ImagePtr gray = cv_bridge::CvImage(std_msgs::Header(), "mono8", depthsGray).toImageMsg();
    depthPub.publish(gray);


}

void getTheoreticalUV(double x, double y, double z, Mat& output) //图像必须提前矫正
{
    double matrix3[4][1] = {x, y, z, 1};

    // transform into the opencv matrix*/
    Mat coordinate(4, 1, CV_64F, matrix3);

    // calculate the result of u and v
    Mat result = camera_matrix*uni_matrix*coordinate;
    float u = result.at<double>(0, 0);
    float v = result.at<double>(1, 0);
    float depth = result.at<double>(2, 0);
    u /= depth;
    v /= depth;
    int x1 = floor(u + 0.5);
    int y1 = floor(v + 0.5);
    if(x1 < imgCols && x1 >= 0 && y1 < imgRows && y1 >= 0)
    {
        output.at<Vec3d>(y1, x1)[0] = x;
        output.at<Vec3d>(y1, x1)[1] = y;
        output.at<Vec3d>(y1, x1)[2] = z;
    }

}
void cloudFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(input);
    sor.setLeafSize(0.05f,0.05f,0.05f);
    sor.filter(*output);
}
void clusterAndSelect(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr& output)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input);
    vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.1);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input);
    ec.extract(clusterIndices);

    int maxPointsNumber = 0;
    int j = 0;

    for (vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin();it!=clusterIndices.end();++it)
    {
        if(it->indices.size() > maxPointsNumber)
        {
            maxPointsNumber = it->indices.size();
            output.reset (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin();pit != it->indices.end();++pit)
            {
                output->points.push_back(input->points[*pit]);
            }
            output->width = output->points.size();
            output->height = 1;
            output->is_dense = true;
        }
    }
}
double getDepthInRect(Rect rect, Mat& depthImg)
{
    depthShow(depthImg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudROI(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ point;
    for(int i = rect.y; i < (rect.y + rect.height); i++)
    {
        for(int j = rect.x; j < (rect.x + rect.width); j++)
        {
            if(depthImg.at<Vec3d>(i, j)[0] != 0)
            {
                point.x = depthImg.at<Vec3d>(i, j)[0];
                point.y = depthImg.at<Vec3d>(i, j)[1];
                point.z = depthImg.at<Vec3d>(i, j)[2];
                cloudROI->points.push_back(point);
            }
        }
    }
    cloudROI->width = cloudROI->points.size();
    cloudROI->height = 1;
    cloudROI->is_dense = true;
    if(cloudROI->size() <= 0)
    {
        cout << "No Livox points in ROI" << rect << endl;
        return 0;
    }
    else
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr mostClusterInROI(new pcl::PointCloud<pcl::PointXYZ>);
        clusterAndSelect(cloudROI, mostClusterInROI);
        double distance = 0;
        if(mostClusterInROI->size() <= 0)
        {
            cout << "No points in cluster selected! The points number in ROI is" << cloudROI->size() << endl;
            return 0;
        }
        else
        {
            for (int i = 0; i < mostClusterInROI->points.size(); i++)
            {
                distance += mostClusterInROI->points[i].x;
            }
            distance /= mostClusterInROI->points.size();
            pointCloudShower(mostClusterInROI);
            return distance;
        }
    }
}
void removeFlat(pcl::PointCloud<pcl::PointXYZ>::Ptr& input)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliners(new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudT(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::SACSegmentation<pcl::PointXYZ> sac;
    sac.setOptimizeCoefficients(true);
    sac.setModelType(pcl::SACMODEL_PLANE);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setDistanceThreshold(0.1);
    sac.setMaxIterations(500);
    int nr_points = (int)input->points.size();
    while(input->points.size() > 0.7 * nr_points)
    {
        sac.setInputCloud(input);
        sac.segment(*inliners, *coefficients);
        if (inliners->indices.size() == 0)
        {
            cout << "could not remove " << endl;
            break;
        }
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(input);
        extract.setIndices(inliners);
        extract.setNegative(true);
        extract.filter(*cloudT);
        *input = *cloudT;
    }
}
void projectPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input, Mat& output)
{
    for (unsigned int i = 0; i < input->size(); ++i)
    {
        getTheoreticalUV(input->points[i].x, input->points[i].y, input->points[i].z, output);
    }
}
double pointCloudShower(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
    static boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("realtime pcl"));
    //cout << input->size() << endl;
    viewer->removeAllPointClouds();  // 移除当前所有点云
    viewer->addPointCloud(input, "realtime pcl");
    viewer->updatePointCloud(input, "realtime pcl");
    viewer->spinOnce(0.001);
}
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    Mat depthes = Mat::zeros(imgRows, imgCols, CV_64FC3);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    cloud.reset (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilted(new pcl::PointCloud<pcl::PointXYZ>);
    //cloudFilter(cloud, cloudFilted);
    removeFlat(cloud);
    projectPoints(cloud, depthes);
    depthShow(depthes);
    Rect rect(320, 180, 130, 80);
    cout << getDepthInRect(rect, depthes) << endl;


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
    depthPub = n.advertise<sensor_msgs::Image>("/depthGray", 1);

    ros::spin();
    return 0;
}