#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <radar_msgs/points.h>
#include <radar_msgs/point.h>
#include <radar_msgs/dist_points.h>

using namespace std;
using namespace cv;

void project(double x, double y, double z, Mat &output, Mat R, Mat T, Mat CamMatrix_); //图像必须提前矫正
cv::Mat img;

float Ky_right = 0.41;
float C_right = 550.0;
float Ky_left = -0.507;
float C_left = -200.53;//x+Ky_left*y-C_left;
int field_width = 28, field_height = 15;
double imgCols = 1280.0, imgRows = 1024.0;
ros::Publisher worldPointPub;
int red_or_blue = 0;//0 is red, 1 is blue

vector<cv::Point3f> far_objectPoints(4);
vector<cv::Point2f> far_imagePoints(4);
cv::Mat far_CamMatrix_ = Mat::zeros(3, 3, CV_64FC1);
cv::Mat far_distCoeffs_ = Mat::zeros(5, 1, CV_64FC1);
Mat far_Rjacob = Mat::zeros(3, 1, CV_64FC1);
Mat far_R = Mat::eye(3, 3, CV_64FC1);
Mat far_T = Mat::zeros(3, 1, CV_64FC1);
int far_calc_flag = 0;

vector<cv::Point3f> close_objectPoints(4);
vector<cv::Point2f> close_imagePoints(4);
cv::Mat close_CamMatrix_ = Mat::zeros(3, 3, CV_64FC1);
cv::Mat close_distCoeffs_ = Mat::zeros(5, 1, CV_64FC1);
Mat close_Rjacob = Mat::zeros(3, 1, CV_64FC1);
Mat close_R = Mat::eye(3, 3, CV_64FC1);
Mat close_T = Mat::zeros(3, 1, CV_64FC1);
int close_calc_flag = 0;
vector<radar_msgs::points> far_points;
vector<radar_msgs::points> close_points;

void far_calibration(const radar_msgs::points &msg);//相机标定
void far_distPointCallback(const radar_msgs::dist_points &input);
void close_calibration(const radar_msgs::points &msg);//相机标定
void close_distPointCallback(const radar_msgs::dist_points &input);

int main(int argc, char **argv) {

    /*solvePnP求解相机外参矩阵*/
    ros::init(argc, argv, "small_map");
    ros::NodeHandle n;

    /*相机标定 Camera Calibration*/
    ros::param::get("/sensor_far/point1/x", far_objectPoints[0].x);
    ros::param::get("/sensor_far/point1/y", far_objectPoints[0].y);
    ros::param::get("/sensor_far/point1/z", far_objectPoints[0].z);
    ros::param::get("/sensor_far/point2/x", far_objectPoints[1].x);
    ros::param::get("/sensor_far/point2/y", far_objectPoints[1].y);
    ros::param::get("/sensor_far/point2/z", far_objectPoints[1].z);
    ros::param::get("/sensor_far/point3/x", far_objectPoints[2].x);
    ros::param::get("/sensor_far/point3/y", far_objectPoints[2].y);
    ros::param::get("/sensor_far/point3/z", far_objectPoints[2].z);
    ros::param::get("/sensor_far/point4/x", far_objectPoints[3].x);
    ros::param::get("/sensor_far/point4/y", far_objectPoints[3].y);
    ros::param::get("/sensor_far/point4/z", far_objectPoints[3].z);
    ros::param::get("/sensor_close/point1/x", close_objectPoints[0].x);
    ros::param::get("/sensor_close/point1/y", close_objectPoints[0].y);
    ros::param::get("/sensor_close/point1/z", close_objectPoints[0].z);
    ros::param::get("/sensor_close/point2/x", close_objectPoints[1].x);
    ros::param::get("/sensor_close/point2/y", close_objectPoints[1].y);
    ros::param::get("/sensor_close/point2/z", close_objectPoints[1].z);
    ros::param::get("/sensor_close/point3/x", close_objectPoints[2].x);
    ros::param::get("/sensor_close/point3/y", close_objectPoints[2].y);
    ros::param::get("/sensor_close/point3/z", close_objectPoints[2].z);
    ros::param::get("/sensor_close/point4/x", close_objectPoints[3].x);
    ros::param::get("/sensor_close/point4/y", close_objectPoints[3].y);
    ros::param::get("/sensor_close/point4/z", close_objectPoints[3].z);
    string btlcolor;
    ros::param::get("/battle_color", btlcolor);
    if (btlcolor == "red")red_or_blue = 0;
    if (btlcolor == "blue") red_or_blue = 1;

    ros::param::get("/sensor_far/camera_matrix/zerozero", far_CamMatrix_.at<double>(0, 0));
    ros::param::get("/sensor_far/camera_matrix/zerotwo", far_CamMatrix_.at<double>(0, 2));
    ros::param::get("/sensor_far/camera_matrix/oneone", far_CamMatrix_.at<double>(1, 1));
    ros::param::get("/sensor_far/camera_matrix/onetwo", far_CamMatrix_.at<double>(1, 2));
    ros::param::get("/sensor_far/camera_matrix/twotwo", far_CamMatrix_.at<double>(2, 2));
    far_CamMatrix_.at<double>(0, 1) = 0;
    far_CamMatrix_.at<double>(1, 0) = 0;
    far_CamMatrix_.at<double>(2, 0) = 0;
    far_CamMatrix_.at<double>(2, 1) = 0;
    cout << far_CamMatrix_ << endl;
    ros::param::get("/sensor_far/distortion_coefficient/zero", far_distCoeffs_.at<double>(0, 0));
    ros::param::get("/sensor_far/distortion_coefficient/one", far_distCoeffs_.at<double>(1, 0));
    ros::param::get("/sensor_far/distortion_coefficient/two", far_distCoeffs_.at<double>(2, 0));
    ros::param::get("/sensor_far/distortion_coefficient/three", far_distCoeffs_.at<double>(3, 0));
    ros::param::get("/sensor_far/distortion_coefficient/four", far_distCoeffs_.at<double>(4, 0));
    cout << far_distCoeffs_ << endl;

    ros::param::get("/sensor_close/camera_matrix/zerozero", close_CamMatrix_.at<double>(0, 0));
    ros::param::get("/sensor_close/camera_matrix/zerotwo", close_CamMatrix_.at<double>(0, 2));
    ros::param::get("/sensor_close/camera_matrix/oneone", close_CamMatrix_.at<double>(1, 1));
    ros::param::get("/sensor_close/camera_matrix/onetwo", close_CamMatrix_.at<double>(1, 2));
    ros::param::get("/sensor_close/camera_matrix/twotwo", close_CamMatrix_.at<double>(2, 2));
    close_CamMatrix_.at<double>(0, 1) = 0;
    close_CamMatrix_.at<double>(1, 0) = 0;
    close_CamMatrix_.at<double>(2, 0) = 0;
    close_CamMatrix_.at<double>(2, 1) = 0;
    cout << close_CamMatrix_ << endl;
    ros::param::get("/sensor_close/distortion_coefficient/zero", close_distCoeffs_.at<double>(0, 0));
    ros::param::get("/sensor_close/distortion_coefficient/one", close_distCoeffs_.at<double>(1, 0));
    ros::param::get("/sensor_close/distortion_coefficient/two", close_distCoeffs_.at<double>(2, 0));
    ros::param::get("/sensor_close/distortion_coefficient/three", close_distCoeffs_.at<double>(3, 0));
    ros::param::get("/sensor_close/distortion_coefficient/four", close_distCoeffs_.at<double>(4, 0));
    cout << close_distCoeffs_ << endl;
    ros::Subscriber far_imageSub = n.subscribe("/sensor_far/calibration", 1, &far_calibration);
    ros::Subscriber far_distPointSub = n.subscribe("/sensor_far/distance_point", 1, &far_distPointCallback);
    ros::Subscriber close_imageSub = n.subscribe("/sensor_close/calibration", 1, &close_calibration);
    ros::Subscriber close_distPointSub = n.subscribe("/sensor_close/distance_point", 1, &close_distPointCallback);
    worldPointPub = n.advertise<radar_msgs::points>("/world_point", 50);
    ros::Rate loop_rate(20);
    Mat small_map;
    if (red_or_blue == 0)small_map = imread("/home/chris/radar_station2022/src/small_map/src/red_minimap.png");
    else small_map = imread("/home/chris/radar_station2022/src/small_map/src/blue_minimap.png");
    while (ros::ok()) {
        ros::spinOnce();
        Mat small_map_copy;
        small_map.copyTo(small_map_copy);
        resize(small_map_copy, small_map_copy, Size(450, 840));
        for (int i = 0; i < far_points.size(); i++) {
            if (far_points[i].color == "red")
                circle(small_map_copy, Point((int) (450 * far_points[i].data[0].x),
                                             (int) (840 * far_points[i].data[0].y)), 10,
                       Scalar(0, 0, 255), -1, LINE_8, 0);
            else if (far_points[i].color == "blue")
                circle(small_map_copy, Point((int) (450 * far_points[i].data[0].x),
                                             (int) (840 * far_points[i].data[0].y)), 10,
                       Scalar(255, 0, 0), -1, LINE_8, 0);
            worldPointPub.publish(far_points[i]);
        }
        for (int i = 0; i < close_points.size(); i++) {
            float x = close_points[i].data[0].x;
            float y = close_points[i].data[0].y;
            x *= 450;
            y *= 840;
            if (x + Ky_right * y - C_right > 0 || x + Ky_left * y - C_left < 0) {
                if (close_points[i].color == "blue")
                    circle(small_map_copy, Point((int) x,
                                                 (int) y), 10,
                           Scalar(255, 0, 0), -1, LINE_8, 0);
                else if (close_points[i].color == "red")
                    circle(small_map_copy, Point((int) x,
                                                 (int) y), 10,
                           Scalar(0, 0, 255), -1, LINE_8, 0);
                worldPointPub.publish(close_points[i]);
            }
        }
        imshow("small_map", small_map_copy);
        waitKey(1);
        loop_rate.sleep();
    }
    return 0;
}

void project(double x, double y, double z, Mat &output, Mat R, Mat T, Mat CamMatrix_) //图像必须提前矫正
{
    double matrix3[4][1] = {x, y, z, 1};//激光雷达系中坐标

    // transform into the opencv matrix*/
    Mat coordinate(4, 1, CV_64F, matrix3);
    Mat uni_matrix(3, 4, CV_64FC1);
    hconcat(R, T, uni_matrix);
    // calculate the result of u and v
    Mat result = CamMatrix_ * uni_matrix * coordinate;
    float u = result.at<double>(0, 0);
    float v = result.at<double>(1, 0);
    float depth = result.at<double>(2, 0);
    u /= depth;
    v /= depth;
    int x1 = floor(u + 0.5);
    int y1 = floor(v + 0.5);
    cout << CamMatrix_ << endl << uni_matrix << endl << coordinate << endl;
    cout << "project point:   " << Point(x1, y1) << endl;
    if (x1 < imgCols && x1 >= 0 && y1 < imgRows && y1 >= 0) {
        circle(output, Point(x1, y1), 2, Scalar(255, 0, 0), 2, 2, 0);
    }
}

void far_distPointCallback(const radar_msgs::dist_points &input) {
    if (far_calc_flag == 1) {
        Mat invR;
        Mat invM;
        invert(far_CamMatrix_, invM);
        invert(far_R, invR);
        std::vector<radar_msgs::points>().swap(far_points);
        for (int i = 0; i < input.data.size(); i++) {
            if (input.data[i].dist > 0) {
                Mat x8_pixel;
                x8_pixel = (Mat_<double>(3, 1) << (double) input.data[i].x, (double) input.data[i].y, 1);
                x8_pixel *= (1000 * input.data[i].dist);
                Mat calcWorld = invR * (invM * x8_pixel - far_T);//2D-3D变换
                calcWorld /= 1000;
                double x = calcWorld.at<double>(0, 0);
                double y = calcWorld.at<double>(1, 0);
                y = field_width - y;
                x /= field_height;
                y /= field_width;
                radar_msgs::point point;
                radar_msgs::points points;
                point.x = x;
                point.y = y;
                points.data.push_back(point);
                points.id = i;
                if (input.data[i].color == 0)points.color = string("red");
                else points.color = string("blue");
                far_points.push_back(points);
            }
        }
    }
}

void far_calibration(const radar_msgs::points &msg) {
    for (const auto &abc: msg.data) {
        far_imagePoints[abc.id] = cv::Point2d(abc.x, abc.y);
        far_imagePoints[abc.id].x *= imgCols;
        far_imagePoints[abc.id].y *= imgRows;
        cout << far_imagePoints[abc.id] << endl;
    }
    cout << "已经选出了4个点!下面进行SolvePnP求解外参矩阵。" << endl;
    cv::Mat abc;
    int suc = cv::solvePnPRansac(far_objectPoints, far_imagePoints, far_CamMatrix_, far_distCoeffs_, far_Rjacob, far_T,
                                 false, 100, 8.0, 0.99,
                                 abc, cv::SOLVEPNP_AP3P);
    Rodrigues(far_Rjacob, far_R); //将R从雅可比形式转换为罗德里格斯形式,输出的R是3x3的一个矩阵。
    cout << "suc:" << suc << endl;
    cout << "旋转矩阵:" << far_R << endl;
    cout << "平移矩阵" << far_T << endl;
    far_calc_flag = 1;
}

void close_distPointCallback(const radar_msgs::dist_points &input) {
    if (close_calc_flag == 1) {
        Mat invR;
        Mat invM;
        invert(close_CamMatrix_, invM);
        invert(close_R, invR);
        std::vector<radar_msgs::points>().swap(close_points);
        for (int i = 0; i < input.data.size(); i++) {
            if (input.data[i].dist > 0) {
                Mat x8_pixel;
                x8_pixel = (Mat_<double>(3, 1) << (double) input.data[i].x, (double) input.data[i].y, 1);
                x8_pixel *= (1000 * input.data[i].dist);
                Mat calcWorld = invR * (invM * x8_pixel - close_T);//2D-3D变换
                calcWorld /= 1000;
                double x = calcWorld.at<double>(0, 0);
                double y = calcWorld.at<double>(1, 0);
                y = field_width - y;
                x /= field_height;
                y /= field_width;
                radar_msgs::point point;
                radar_msgs::points points;
                point.x = x;
                point.y = y;
                points.data.push_back(point);
                points.id = i;
                if (input.data[i].color == 0)points.color = string("red");
                else points.color = string("blue");
                close_points.push_back(points);
            }
        }
    }
}

void close_calibration(const radar_msgs::points &msg) {
    for (const auto &abc: msg.data) {
        close_imagePoints[abc.id] = cv::Point2d(abc.x, abc.y);
        close_imagePoints[abc.id].x *= imgCols;
        close_imagePoints[abc.id].y *= imgRows;
        cout << close_imagePoints[abc.id] << endl;
    }
    cout << "已经选出了4个点!下面进行SolvePnP求解外参矩阵。" << endl;
    cv::Mat abc;
    int suc = cv::solvePnPRansac(close_objectPoints, close_imagePoints, close_CamMatrix_, close_distCoeffs_,
                                 close_Rjacob, close_T, false, 100, 8.0, 0.99,
                                 abc, cv::SOLVEPNP_AP3P);
    Rodrigues(close_Rjacob, close_R); //将R从雅可比形式转换为罗德里格斯形式,输出的R是3x3的一个矩阵。
    cout << "suc:" << suc << endl;
    cout << "旋转矩阵:" << close_R << endl;
    cout << "平移矩阵" << close_T << endl;
    close_calc_flag = 1;
}
