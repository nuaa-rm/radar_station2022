//
// Created by dovejh on 2022/3/19.
//
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <radar_msgs/dist_point.h>
#include <radar_msgs/dist_points.h>
#include <radar_msgs/yolo_points.h>

using namespace std;
using namespace cv;

int imgRows = 1024, imgCols = 1280;

ros::Publisher far_distancePointPub;
ros::Publisher close_distancePointPub;
radar_msgs::dist_points far_distance_it;
radar_msgs::dist_points close_distance_it;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
Mat far_depthes = Mat::zeros(imgRows, imgCols, CV_64FC3);//initialize the depth img
Mat close_depthes = Mat::zeros(imgRows, imgCols, CV_64FC3);//initialize the depth img
Mat far_camera_matrix = Mat_<double>(3, 3);//相机内参矩阵
Mat far_uni_matrix = Mat_<double>(3, 4);//相机和雷达的变换矩阵
Mat far_distortion_coefficient = Mat_<double>(5, 1);
Mat close_camera_matrix = Mat_<double>(3, 3);//相机内参矩阵
Mat close_uni_matrix = Mat_<double>(3, 4);//相机和雷达的变换矩阵
Mat close_distortion_coefficient = Mat_<double>(5, 1);
uint8_t i = 0;//the number of car_points vector
void getTheoreticalUV(double x, double y, double z, Mat Cam_matrix, Mat Uni_matrix, Mat &output);//得到某一点对应图像中的位置

double getDepthInRect(Rect rect, Mat &depthImg);//得到ROI中点的深度
void projectPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input, Mat Cam_matrix, Mat Uni_matrix,
                   Mat &output);//对于每一个点云中的点调用一次getTheoreticalUV函数
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &input);

void far_yoloCallback(const radar_msgs::yolo_points::ConstPtr &input);

void close_yoloCallback(const radar_msgs::yolo_points::ConstPtr &input);

void getTheoreticalUV(double x, double y, double z, Mat Cam_matrix, Mat Uni_matrix, Mat &output) //图像必须提前矫正
{
    double matrix3[4][1] = {x, y, z, 1};//激光雷达系中坐标

    // transform into the opencv matrix*/
    Mat coordinate(4, 1, CV_64F, matrix3);

    // calculate the result of u and v
    Mat result = Cam_matrix * Uni_matrix * coordinate;
    float u = result.at<double>(0, 0);
    float v = result.at<double>(1, 0);
    float depth = result.at<double>(2, 0);
    u /= depth;
    v /= depth;
    int x1 = floor(u + 0.5);
    int y1 = floor(v + 0.5);
    if (x1 < imgCols && x1 >= 0 && y1 < imgRows && y1 >= 0) {
        output.at<Vec3d>(y1, x1)[0] = x;
        output.at<Vec3d>(y1, x1)[1] = y;
        output.at<Vec3d>(y1, x1)[2] = z;
    }

}

double getDepthInRect(Rect rect, Mat &depthImg) {
    vector<float> distances;
    for (int i = rect.y; i < (rect.y + rect.height); i++) {
        for (int j = rect.x; j < (rect.x + rect.width); j++) {
            if (depthImg.at<Vec3d>(i, j)[0] != 0) {
                distances.push_back(depthImg.at<Vec3d>(i, j)[0]);
            }
        }
    }
    if (distances.empty()) {
        cout << "No Livox points in ROI" << rect << endl;
        return 0;
    } else {
        sort(distances.begin(), distances.end());
        int position=distances.size()/2;
        float mean_distance=0;
        if(position!=0){
            for(int i=0;i<position;i++){
                mean_distance+=distances[i];
            }
            mean_distance/=position;
        }
        return mean_distance;
    }
}

void projectPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input, Mat Cam_matrix, Mat Uni_matrix, Mat &output) {
    for (unsigned int i = 0; i < input->size(); ++i) {
        getTheoreticalUV(input->points[i].x, input->points[i].y, input->points[i].z, Cam_matrix, Uni_matrix, output);
    }
}

//update the dethes_img by pointcloud
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &input) {
    //obtain the depth image by project
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);
}

//update the car_rects
void far_yoloCallback(const radar_msgs::yolo_points::ConstPtr &input) {
    far_depthes = Mat::zeros(imgRows, imgCols, CV_64FC3);//initialize the depth img
    std::vector<radar_msgs::dist_point>().swap(far_distance_it.data);
    if(cloud)projectPoints(cloud, far_camera_matrix, far_uni_matrix, far_depthes);
    if ((*input).text != "none") {
//        close_distance_it.color = (*input).color;
        for (int j = 0; j < (*input).data.size(); j++) {
            radar_msgs::dist_point point_it;
            point_it.x = (*input).data[j].x + (*input).data[j].width / 2;
            point_it.y = (*input).data[j].y + (*input).data[j].height / 2;
            point_it.dist = getDepthInRect(
                    Rect((*input).data[j].x, (*input).data[j].y, (*input).data[j].width, (*input).data[j].height),
                    far_depthes);
            point_it.color=(*input).data[j].color;
            point_it.id = j;
            rectangle(far_depthes,
                      Rect((*input).data[j].x, (*input).data[j].y, (*input).data[j].width, (*input).data[j].height),
                      Scalar(255, 255, 255), 1);
            putText(far_depthes, std::to_string(point_it.dist), Point(point_it.x, point_it.y),
                    FONT_HERSHEY_COMPLEX_SMALL, 1,
                    Scalar(255, 255, 255), 1, 8, 0);
            far_distance_it.data.push_back(point_it);
        }
    }
    far_distancePointPub.publish(far_distance_it);
    imshow("far_depthes", far_depthes);
    waitKey(1);
}

//update the car_rects
void close_yoloCallback(const radar_msgs::yolo_points::ConstPtr &input) {
    close_depthes = Mat::zeros(imgRows, imgCols, CV_64FC3);//initialize the depth img
    std::vector<radar_msgs::dist_point>().swap(close_distance_it.data);
    if(cloud)projectPoints(cloud, close_camera_matrix, close_uni_matrix, close_depthes);
    if ((*input).text != "none") {
        for (int j = 0; j < (*input).data.size(); j++) {
            radar_msgs::dist_point point_it;
            point_it.x = (*input).data[j].x + (*input).data[j].width / 2;
            point_it.y = (*input).data[j].y + (*input).data[j].height / 2;
            point_it.dist = getDepthInRect(
                    Rect((*input).data[j].x, (*input).data[j].y, (*input).data[j].width, (*input).data[j].height),
                    close_depthes);
            point_it.color=(*input).data[j].color;
            point_it.id = j;
            rectangle(close_depthes,
                      Rect((*input).data[j].x, (*input).data[j].y, (*input).data[j].width, (*input).data[j].height),
                      Scalar(255, 255, 255), 1);
            putText(close_depthes, std::to_string(point_it.dist), Point(point_it.x, point_it.y),
                    FONT_HERSHEY_COMPLEX_SMALL, 1,
                    Scalar(255, 255, 255), 1, 8, 0);
            close_distance_it.data.push_back(point_it);
        }
    }
    close_distancePointPub.publish(close_distance_it);
    imshow("close_depthes", close_depthes);
    waitKey(1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "get_depth_node");
    ros::NodeHandle n;

    ros::param::get("/image_width", imgCols);
    ros::param::get("/image_height", imgRows);

    ros::param::get("/sensor_far/camera_matrix/zerozero", far_camera_matrix.at<double>(0, 0));
    ros::param::get("/sensor_far/camera_matrix/zerotwo", far_camera_matrix.at<double>(0, 2));
    ros::param::get("/sensor_far/camera_matrix/oneone", far_camera_matrix.at<double>(1, 1));
    ros::param::get("/sensor_far/camera_matrix/onetwo", far_camera_matrix.at<double>(1, 2));
    ros::param::get("/sensor_far/camera_matrix/twotwo", far_camera_matrix.at<double>(2, 2));
    far_camera_matrix.at<double>(0, 1) = 0;
    far_camera_matrix.at<double>(1, 0) = 0;
    far_camera_matrix.at<double>(2, 0) = 0;
    far_camera_matrix.at<double>(2, 1) = 0;
    cout << "far_Camera matrix load done!" << far_camera_matrix << endl;
    ros::param::get("/sensor_far/distortion_coefficient/zero", far_distortion_coefficient.at<double>(0, 0));
    ros::param::get("/sensor_far/distortion_coefficient/one", far_distortion_coefficient.at<double>(1, 0));
    ros::param::get("/sensor_far/distortion_coefficient/two", far_distortion_coefficient.at<double>(2, 0));
    ros::param::get("/sensor_far/distortion_coefficient/three", far_distortion_coefficient.at<double>(3, 0));
    ros::param::get("/sensor_far/distortion_coefficient/four", far_distortion_coefficient.at<double>(4, 0));
    cout << "far_Distortion coefficient load done!" << far_distortion_coefficient << endl;
    ros::param::get("/sensor_far/uni_matrix/zerozero", far_uni_matrix.at<double>(0, 0));
    ros::param::get("/sensor_far/uni_matrix/zeroone", far_uni_matrix.at<double>(0, 1));
    ros::param::get("/sensor_far/uni_matrix/zerotwo", far_uni_matrix.at<double>(0, 2));
    ros::param::get("/sensor_far/uni_matrix/zerothree", far_uni_matrix.at<double>(0, 3));
    ros::param::get("/sensor_far/uni_matrix/onezero", far_uni_matrix.at<double>(1, 0));
    ros::param::get("/sensor_far/uni_matrix/oneone", far_uni_matrix.at<double>(1, 1));
    ros::param::get("/sensor_far/uni_matrix/onetwo", far_uni_matrix.at<double>(1, 2));
    ros::param::get("/sensor_far/uni_matrix/onethree", far_uni_matrix.at<double>(1, 3));
    ros::param::get("/sensor_far/uni_matrix/twozero", far_uni_matrix.at<double>(2, 0));
    ros::param::get("/sensor_far/uni_matrix/twoone", far_uni_matrix.at<double>(2, 1));
    ros::param::get("/sensor_far/uni_matrix/twotwo", far_uni_matrix.at<double>(2, 2));
    ros::param::get("/sensor_far/uni_matrix/twothree", far_uni_matrix.at<double>(2, 3));
    cout << "far Uni matrix load done!" << far_uni_matrix << endl;


    ros::param::get("/sensor_close/camera_matrix/zerozero", close_camera_matrix.at<double>(0, 0));
    ros::param::get("/sensor_close/camera_matrix/zerotwo", close_camera_matrix.at<double>(0, 2));
    ros::param::get("/sensor_close/camera_matrix/oneone", close_camera_matrix.at<double>(1, 1));
    ros::param::get("/sensor_close/camera_matrix/onetwo", close_camera_matrix.at<double>(1, 2));
    ros::param::get("/sensor_close/camera_matrix/twotwo", close_camera_matrix.at<double>(2, 2));
    close_camera_matrix.at<double>(0, 1) = 0;
    close_camera_matrix.at<double>(1, 0) = 0;
    close_camera_matrix.at<double>(2, 0) = 0;
    close_camera_matrix.at<double>(2, 1) = 0;
    cout << "close_Camera matrix load done!" << close_camera_matrix << endl;
    ros::param::get("/sensor_close/distortion_coefficient/zero", close_distortion_coefficient.at<double>(0, 0));
    ros::param::get("/sensor_close/distortion_coefficient/one", close_distortion_coefficient.at<double>(1, 0));
    ros::param::get("/sensor_close/distortion_coefficient/two", close_distortion_coefficient.at<double>(2, 0));
    ros::param::get("/sensor_close/distortion_coefficient/three", close_distortion_coefficient.at<double>(3, 0));
    ros::param::get("/sensor_close/distortion_coefficient/four", close_distortion_coefficient.at<double>(4, 0));
    cout << "close_Distortion coefficient load done!" << close_distortion_coefficient << endl;
    ros::param::get("/sensor_close/uni_matrix/zerozero", close_uni_matrix.at<double>(0, 0));
    ros::param::get("/sensor_close/uni_matrix/zeroone", close_uni_matrix.at<double>(0, 1));
    ros::param::get("/sensor_close/uni_matrix/zerotwo", close_uni_matrix.at<double>(0, 2));
    ros::param::get("/sensor_close/uni_matrix/zerothree", close_uni_matrix.at<double>(0, 3));
    ros::param::get("/sensor_close/uni_matrix/onezero", close_uni_matrix.at<double>(1, 0));
    ros::param::get("/sensor_close/uni_matrix/oneone", close_uni_matrix.at<double>(1, 1));
    ros::param::get("/sensor_close/uni_matrix/onetwo", close_uni_matrix.at<double>(1, 2));
    ros::param::get("/sensor_close/uni_matrix/onethree", close_uni_matrix.at<double>(1, 3));
    ros::param::get("/sensor_close/uni_matrix/twozero", close_uni_matrix.at<double>(2, 0));
    ros::param::get("/sensor_close/uni_matrix/twoone", close_uni_matrix.at<double>(2, 1));
    ros::param::get("/sensor_close/uni_matrix/twotwo", close_uni_matrix.at<double>(2, 2));
    ros::param::get("/sensor_close/uni_matrix/twothree", close_uni_matrix.at<double>(2, 3));
    cout << "close Uni matrix load done!" << close_uni_matrix << endl;

    ros::Subscriber cloud_sub;
    cloud_sub = n.subscribe("/livox/lidar", 1, &pointCloudCallback);
    ros::Subscriber far_yolo_sub;
    far_yolo_sub = n.subscribe("/far_rectangles", 1, &far_yoloCallback);
    ros::Subscriber close_yolo_sub;
    close_yolo_sub = n.subscribe("/close_rectangles", 1, &close_yoloCallback);
    far_distancePointPub = n.advertise<radar_msgs::dist_points>("/sensor_far/distance_point", 1);
    close_distancePointPub = n.advertise<radar_msgs::dist_points>("/sensor_close/distance_point", 1);
    ros::Rate loop_rate(30);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
