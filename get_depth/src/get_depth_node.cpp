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
#include <numeric>

using namespace std;
using namespace cv;

int imgRows = 1024, imgCols = 1280;
int length_of_cloud_queue = 5;//default length is 5

ros::Publisher far_distancePointPub;
ros::Publisher close_distancePointPub;
radar_msgs::dist_points far_distance_it;
radar_msgs::dist_points close_distance_it;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
vector<Mat> far_depth_queue;
Mat far_camera_matrix = Mat_<float>(3, 3);//相机内参矩阵
Mat far_uni_matrix = Mat_<float>(3, 4);//相机和雷达的变换矩阵
Mat far_distortion_coefficient = Mat_<float>(5, 1);
vector<Mat> close_depth_queue;
Mat close_camera_matrix = Mat_<float>(3, 3);//相机内参矩阵
Mat close_uni_matrix = Mat_<float>(3, 4);//相机和雷达的变换矩阵
Mat close_distortion_coefficient = Mat_<float>(5, 1);

float getDepthInRect(Rect rect, vector<Mat> &depth_queue, radar_msgs::yolo_point::_id_type id);//得到ROI中点的深度

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &input);

void far_yoloCallback(const radar_msgs::yolo_points::ConstPtr &input);

void close_yoloCallback(const radar_msgs::yolo_points::ConstPtr &input);

Mat Cloud2Mat(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input);//Convert PointCloud to Mat

void
MatProject(Mat &input_depth, Mat &input_uv, Mat &Cam_matrix,
           Mat &Uni_matrix);//Convert world to uv by Matrix_Calculation

Mat Cloud2Mat(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input) {
    Mat res = Mat::zeros(4, (int) input->size(), CV_32F);
    for (int k = 0; k < res.cols; k++) {
        for (int j = 0; j < 4; j++) {
            res.at<float>(j, k) = input->points[k].data[j];
        }
    }
    return res;
}

void MatProject(Mat &input_depth, Mat &input_uv, Mat &Cam_matrix, Mat &Uni_matrix) {
    Mat res = Cam_matrix * Uni_matrix * input_uv;
    for (int i = 0; i < res.cols; i++) {
        int x = round(res.at<float>(0, i) / res.at<float>(2, i));
        int y = round(res.at<float>(1, i) / res.at<float>(2, i));
        if (x >= 0 && x < imgCols && y >= 0 && y < imgRows) {
            input_depth.at<float>(y, x) = res.at<float>(2, i);
        }
    }
}

//update the car_rects
void far_yoloCallback(const radar_msgs::yolo_points::ConstPtr &input) {
    Mat far_depthes = Mat::zeros(imgRows, imgCols, CV_32FC1);//initialize the depth img
    Mat far_depth_show = Mat::zeros(imgRows, imgCols, CV_32FC1);
    std::vector<radar_msgs::dist_point>().swap(far_distance_it.data);
    if (cloud) {
        Mat far_MatCloud = Cloud2Mat(cloud);
        MatProject(far_depthes, far_MatCloud, far_camera_matrix, far_uni_matrix);
        far_depthes.copyTo(far_depth_show);
        far_depth_queue.push_back(far_depthes);
        if (far_depth_queue.size() == length_of_cloud_queue) {
            far_depth_queue.erase(far_depth_queue.begin());
        }
    }
    if ((*input).text != "none") {
        for (int j = 0; j < (*input).data.size(); j++) {
            radar_msgs::dist_point point_it;
            point_it.x = (*input).data[j].x + (*input).data[j].width / 2;
            point_it.y = (*input).data[j].y + (*input).data[j].height / 2;
            point_it.dist = getDepthInRect(
                    Rect((*input).data[j].x, (*input).data[j].y, (*input).data[j].width, (*input).data[j].height),
                    far_depth_queue,(*input).data[j].id);
            point_it.color = (*input).data[j].color;
            point_it.id = (*input).data[j].id;
            far_distance_it.data.push_back(point_it);
            rectangle(far_depth_show,
                      Rect((*input).data[j].x, (*input).data[j].y, (*input).data[j].width, (*input).data[j].height),
                      Scalar(255, 255, 255), 1);
            putText(far_depth_show, std::to_string(point_it.dist), Point(point_it.x, point_it.y),
                    FONT_HERSHEY_COMPLEX_SMALL, 1,
                    Scalar(255, 255, 255), 1, 8, 0);
        }
    }
    far_distancePointPub.publish(far_distance_it);
    resize(far_depth_show, far_depth_show, Size(960, 768));
//    imshow("far_depth_show", far_depth_show);
//    waitKey(1);
}

//update the car_rects
void close_yoloCallback(const radar_msgs::yolo_points::ConstPtr &input) {
    Mat close_depthes = Mat::zeros(imgRows, imgCols, CV_32FC1);//initialize the depth img
    Mat close_depth_show = Mat::zeros(imgRows, imgCols, CV_32FC1);
    std::vector<radar_msgs::dist_point>().swap(close_distance_it.data);
    if (cloud) {
        Mat close_MatCloud = Cloud2Mat(cloud);
        MatProject(close_depthes, close_MatCloud, close_camera_matrix, close_uni_matrix);
        close_depthes.copyTo(close_depth_show);
        close_depth_queue.push_back(close_depthes);
        if (close_depth_queue.size() == length_of_cloud_queue) {
            close_depth_queue.erase(close_depth_queue.begin());
        }
    }
    if ((*input).text != "none") {
        for (int j = 0; j < (*input).data.size(); j++) {
            radar_msgs::dist_point point_it;
            point_it.x = (*input).data[j].x + (*input).data[j].width / 2;
            point_it.y = (*input).data[j].y + (*input).data[j].height / 2;
            point_it.dist = getDepthInRect(
                    Rect((*input).data[j].x, (*input).data[j].y, (*input).data[j].width, (*input).data[j].height),
                    close_depth_queue, (*input).data[j].id);
            point_it.color = (*input).data[j].color;
            point_it.id = (*input).data[j].id;
            close_distance_it.data.push_back(point_it);
            close_distance_it.data.push_back(point_it);
            rectangle(close_depth_show,
                      Rect((*input).data[j].x, (*input).data[j].y, (*input).data[j].width, (*input).data[j].height),
                      Scalar(255, 255, 255), 1);
            putText(close_depth_show, std::to_string(point_it.dist), Point(point_it.x, point_it.y),
                    FONT_HERSHEY_COMPLEX_SMALL, 1,
                    Scalar(255, 255, 255), 1, 8, 0);
        }
    }
    close_distancePointPub.publish(close_distance_it);
    resize(close_depth_show, close_depth_show, Size(960, 768));
//    imshow("close_depth_show", close_depth_show);
//    waitKey(1);
}

//update the dethes_img by pointcloud
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &input) {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);
}

float getDepthInRect(Rect rect, vector<Mat> &depth_queue, radar_msgs::yolo_point::_id_type id) {
    vector<float> distances;
    for (int i = rect.y; i < (rect.y + rect.height); i++) {
        for (int j = rect.x; j < (rect.x + rect.width); j++) {
            for (uint8_t k = 0; k < depth_queue.size(); k++) {
                if (depth_queue[depth_queue.size() - 1].at<float>(i, j) > 0) {
                    distances.push_back(depth_queue[depth_queue.size() - 1].at<float>(i, j));
                    break;
                } else if (k < depth_queue.size() - 1 && depth_queue[k + 1].at<float>(i, j) == 0 &&
                           depth_queue[k].at<float>(i, j) > 0) {
                    distances.push_back(depth_queue[k].at<float>(i, j));
                    break;
                }
            }
        }
    }
    if (distances.empty()) {
        cout << "No Livox points in ROI" << rect << endl;
        return 0;
    } else {
        float mean_distance;
        float sum=0;
        if (id != 12 && id != 13) {
            for(float distance : distances){
                sum+=distance;
            }
            mean_distance = sum/distances.size();
            return mean_distance;
        } else {
            sort(distances.begin(), distances.end());
            if (distances.size() >= 5){
                for(uint8_t j=0;j<5;j++){
                    sum+=distances[j];
                }
                mean_distance=sum/5;
                return mean_distance;
            }
            else {
                return distances[0];
            }
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "get_depth_node");
    ros::NodeHandle n;

    ros::param::get("/length_of_cloud_queue", length_of_cloud_queue);
    ros::param::get("/image_width", imgCols);
    ros::param::get("/image_height", imgRows);
    ros::param::get("/sensor_far/camera_matrix/zerozero", far_camera_matrix.at<float>(0, 0));
    ros::param::get("/sensor_far/camera_matrix/zerotwo", far_camera_matrix.at<float>(0, 2));
    ros::param::get("/sensor_far/camera_matrix/oneone", far_camera_matrix.at<float>(1, 1));
    ros::param::get("/sensor_far/camera_matrix/onetwo", far_camera_matrix.at<float>(1, 2));
    ros::param::get("/sensor_far/camera_matrix/twotwo", far_camera_matrix.at<float>(2, 2));
    far_camera_matrix.at<float>(0, 1) = 0;
    far_camera_matrix.at<float>(1, 0) = 0;
    far_camera_matrix.at<float>(2, 0) = 0;
    far_camera_matrix.at<float>(2, 1) = 0;
    cout << "far_Camera matrix load done!" << far_camera_matrix << endl;
    ros::param::get("/sensor_far/distortion_coefficient/zero", far_distortion_coefficient.at<float>(0, 0));
    ros::param::get("/sensor_far/distortion_coefficient/one", far_distortion_coefficient.at<float>(1, 0));
    ros::param::get("/sensor_far/distortion_coefficient/two", far_distortion_coefficient.at<float>(2, 0));
    ros::param::get("/sensor_far/distortion_coefficient/three", far_distortion_coefficient.at<float>(3, 0));
    ros::param::get("/sensor_far/distortion_coefficient/four", far_distortion_coefficient.at<float>(4, 0));
    cout << "far_Distortion coefficient load done!" << far_distortion_coefficient << endl;
    ros::param::get("/sensor_far/uni_matrix/zerozero", far_uni_matrix.at<float>(0, 0));
    ros::param::get("/sensor_far/uni_matrix/zeroone", far_uni_matrix.at<float>(0, 1));
    ros::param::get("/sensor_far/uni_matrix/zerotwo", far_uni_matrix.at<float>(0, 2));
    ros::param::get("/sensor_far/uni_matrix/zerothree", far_uni_matrix.at<float>(0, 3));
    ros::param::get("/sensor_far/uni_matrix/onezero", far_uni_matrix.at<float>(1, 0));
    ros::param::get("/sensor_far/uni_matrix/oneone", far_uni_matrix.at<float>(1, 1));
    ros::param::get("/sensor_far/uni_matrix/onetwo", far_uni_matrix.at<float>(1, 2));
    ros::param::get("/sensor_far/uni_matrix/onethree", far_uni_matrix.at<float>(1, 3));
    ros::param::get("/sensor_far/uni_matrix/twozero", far_uni_matrix.at<float>(2, 0));
    ros::param::get("/sensor_far/uni_matrix/twoone", far_uni_matrix.at<float>(2, 1));
    ros::param::get("/sensor_far/uni_matrix/twotwo", far_uni_matrix.at<float>(2, 2));
    ros::param::get("/sensor_far/uni_matrix/twothree", far_uni_matrix.at<float>(2, 3));
    cout << "far Uni matrix load done!" << far_uni_matrix << endl;


    ros::param::get("/sensor_close/camera_matrix/zerozero", close_camera_matrix.at<float>(0, 0));
    ros::param::get("/sensor_close/camera_matrix/zerotwo", close_camera_matrix.at<float>(0, 2));
    ros::param::get("/sensor_close/camera_matrix/oneone", close_camera_matrix.at<float>(1, 1));
    ros::param::get("/sensor_close/camera_matrix/onetwo", close_camera_matrix.at<float>(1, 2));
    ros::param::get("/sensor_close/camera_matrix/twotwo", close_camera_matrix.at<float>(2, 2));
    close_camera_matrix.at<float>(0, 1) = 0;
    close_camera_matrix.at<float>(1, 0) = 0;
    close_camera_matrix.at<float>(2, 0) = 0;
    close_camera_matrix.at<float>(2, 1) = 0;
    cout << "close_Camera matrix load done!" << close_camera_matrix << endl;
    ros::param::get("/sensor_close/distortion_coefficient/zero", close_distortion_coefficient.at<float>(0, 0));
    ros::param::get("/sensor_close/distortion_coefficient/one", close_distortion_coefficient.at<float>(1, 0));
    ros::param::get("/sensor_close/distortion_coefficient/two", close_distortion_coefficient.at<float>(2, 0));
    ros::param::get("/sensor_close/distortion_coefficient/three", close_distortion_coefficient.at<float>(3, 0));
    ros::param::get("/sensor_close/distortion_coefficient/four", close_distortion_coefficient.at<float>(4, 0));
    cout << "close_Distortion coefficient load done!" << close_distortion_coefficient << endl;
    ros::param::get("/sensor_close/uni_matrix/zerozero", close_uni_matrix.at<float>(0, 0));
    ros::param::get("/sensor_close/uni_matrix/zeroone", close_uni_matrix.at<float>(0, 1));
    ros::param::get("/sensor_close/uni_matrix/zerotwo", close_uni_matrix.at<float>(0, 2));
    ros::param::get("/sensor_close/uni_matrix/zerothree", close_uni_matrix.at<float>(0, 3));
    ros::param::get("/sensor_close/uni_matrix/onezero", close_uni_matrix.at<float>(1, 0));
    ros::param::get("/sensor_close/uni_matrix/oneone", close_uni_matrix.at<float>(1, 1));
    ros::param::get("/sensor_close/uni_matrix/onetwo", close_uni_matrix.at<float>(1, 2));
    ros::param::get("/sensor_close/uni_matrix/onethree", close_uni_matrix.at<float>(1, 3));
    ros::param::get("/sensor_close/uni_matrix/twozero", close_uni_matrix.at<float>(2, 0));
    ros::param::get("/sensor_close/uni_matrix/twoone", close_uni_matrix.at<float>(2, 1));
    ros::param::get("/sensor_close/uni_matrix/twotwo", close_uni_matrix.at<float>(2, 2));
    ros::param::get("/sensor_close/uni_matrix/twothree", close_uni_matrix.at<float>(2, 3));
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
