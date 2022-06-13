#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <radar_msgs/relative_coordinate.h>
#include <radar_msgs/points.h>
#include <radar_msgs/point.h>
#include <radar_msgs/distance_point.h>
#include <radar_msgs/world_point.h>

using namespace std;
using namespace cv;
int field_width = 28, field_height = 15;
double imgCols = 1280.0, imgRows = 1024.0;
int i = 0;
ros::Publisher worldPointPub;
radar_msgs::points worldPoint;
radar_msgs::point point;
cv::Mat img;
int red_or_blue=0;//0 is red, 1 is blue
int cnt = 0;
//世界坐标,取四个点用于solvePnP的objects_array
vector<cv::Point3f> objectPoints(4);
//像平面坐标,鼠标在图像上选点并用于solvePnP的imagePoints
vector<cv::Point2f> imagePoints(4);
vector<cv::Point2f> reprojectPoints(4);
vector<string> imagePoints_string(4);
//相机内参矩阵
// cv::Mat CamMatrix_ = (cv::Mat_<double>(3,3) << 919.1319, 0, 645.5211, 0, 919.7624, 539.8345, 0, 0, 1);
//cv::Mat CamMatrix_ = (cv::Mat_<double>(3, 3) << 865.4454, 0, 307.764, 0, 865.9519, 221.537, 0, 0, 1);
cv::Mat CamMatrix_ = Mat::zeros(3, 3, CV_64FC1);
//是否求出外参矩阵的标志位
bool if_pnp = false;
//旋转矩阵和平移矩阵
//cv::Mat Rjacob,R,T;
Mat Rjacob = Mat::zeros(3, 1, CV_64FC1);
Mat R = Mat::eye(3, 3, CV_64FC1);
Mat T = Mat::zeros(3, 1, CV_64FC1);
//参数求解标志位
int calc_flag = 0;
int cout_flag = 0;
//相机畸变系数k1、k2、p1、p2、k3
cv::Mat distCoeffs_ = Mat::zeros(5, 1, CV_64FC1);
string param_name;

void onMouse(int event, int x, int y, int flags, void *ustc);//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
void imageCB(const sensor_msgs::ImageConstPtr &msg);

void calibration(const radar_msgs::points &msg);//相机标定
void reproject(void);//反投影
void depthShow(Mat &input);//将深度图像归一化成灰度图并发布话题进行展示
void distPointCallback(const radar_msgs::points &input);

void project(double x, double y, double z, Mat &output, Mat R, Mat T, Mat CamMatrix_); //图像必须提前矫正

int main(int argc, char **argv) {

    /*solvePnP求解相机外参矩阵*/
//    ros::init(argc, argv, "map_close");
//    ros::NodeHandle n("/sensor_close");
    ros::init(argc, argv, "small_map");
    ros::NodeHandle n;

    /*相机标定 Camera Calibration*/
    ros::param::get("/point1/x", objectPoints[0].x);
    ros::param::get("/point1/y", objectPoints[0].y);
    ros::param::get("/point1/z", objectPoints[0].z);
    ros::param::get("/point2/x", objectPoints[1].x);
    ros::param::get("/point2/y", objectPoints[1].y);
    ros::param::get("/point2/z", objectPoints[1].z);
    ros::param::get("/point3/x", objectPoints[2].x);
    ros::param::get("/point3/y", objectPoints[2].y);
    ros::param::get("/point3/z", objectPoints[2].z);
    ros::param::get("/point4/x", objectPoints[3].x);
    ros::param::get("/point4/y", objectPoints[3].y);
    ros::param::get("/point4/z", objectPoints[3].z);
    string btlcolor;
    ros::param::get("/battle_color", btlcolor);
    if(btlcolor=="red")red_or_blue=0;
    if(btlcolor=="blue") red_or_blue=1;
//    ros::param::get("/point5/x", objectPoints[4].x);
//    ros::param::get("/point5/y", objectPoints[4].y);
//    ros::param::get("/point5/z", objectPoints[4].z);
    int id = 0;
    param_name = n.getNamespace();
    ros::param::get(param_name + "/camera_matrix/zerozero", CamMatrix_.at<double>(0, 0));
    ros::param::get(param_name + "/camera_matrix/zerotwo", CamMatrix_.at<double>(0, 2));
    ros::param::get(param_name + "/camera_matrix/oneone", CamMatrix_.at<double>(1, 1));
    ros::param::get(param_name + "/camera_matrix/onetwo", CamMatrix_.at<double>(1, 2));
    ros::param::get(param_name + "/camera_matrix/twotwo", CamMatrix_.at<double>(2, 2));
    CamMatrix_.at<double>(0, 1) = 0;
    CamMatrix_.at<double>(1, 0) = 0;
    CamMatrix_.at<double>(2, 0) = 0;
    CamMatrix_.at<double>(2, 1) = 0;
    cout << CamMatrix_ << endl;
    ros::param::get(param_name + "/distortion_coefficient/zero", distCoeffs_.at<double>(0, 0));
    ros::param::get(param_name + "/distortion_coefficient/one", distCoeffs_.at<double>(1, 0));
    ros::param::get(param_name + "/distortion_coefficient/two", distCoeffs_.at<double>(2, 0));
    ros::param::get(param_name + "/distortion_coefficient/three", distCoeffs_.at<double>(3, 0));
    ros::param::get(param_name + "/distortion_coefficient/four", distCoeffs_.at<double>(4, 0));
    cout << distCoeffs_ << endl;
    // ros::Subscriber msg_sub = n.subscribe("relative_coordinate", 100, msgCallback);
//    ros::Subscriber imagesub = n.subscribe(param_name + "/image_raw", 1, &imageCB);
    ros::Subscriber imageSub = n.subscribe(param_name + "/calibration", 1, &calibration);
    ros::Subscriber distPointSub = n.subscribe(param_name+"/distance_point", 100, &distPointCallback);
//    ros::Subscriber distPointSub = n.subscribe("/dist", 10, &distPointCallback);
    worldPointPub = n.advertise<radar_msgs::points>("/world_point", 4);
    ros::Rate loop_rate(10);
    while (ros::ok()) {
//        if (!worldPoint.data.empty()) {
//            worldPoint.id = 0;
//            worldPoint.color = string("red");
//            worldPointPub.publish(worldPoint);
//            cout << worldPoint.data.size() << endl;
//            cout << worldPoint.data[0].x << "  " << worldPoint.data[0].y << endl;
//            std::vector<radar_msgs::point>().swap(worldPoint.data);
//        }
        ros::spinOnce();
        loop_rate.sleep();
    }
//    ros::spin();
    return 0;
}


void onMouse(int event, int x, int y, int flags, void *ustc)//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
{
    if (event == cv::EVENT_LBUTTONDOWN && !img.empty())//左键按下，读取初始坐标，并在图像上该点处划圆
    {
        char temp1[16];
        sprintf(temp1, "(%d,%d)", x, y);
        string temp = temp1;
        if (cnt <= 3)
            imagePoints_string[cnt] = temp;
        reprojectPoints[cnt] = cv::Point2f((double) x, (double) y);
        cnt++;
    }
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

void distPointCallback(const radar_msgs::points &input) {
    if (calc_flag == 1&&input.data[2].x!=0) {
        Mat invR;
        Mat invM;
        invert(CamMatrix_, invM);
        invert(R, invR);
        Mat x8_pixel;
        x8_pixel = (Mat_<double>(3, 1) << (double) (input.data[0].x + input.data[1].x) / 2,
                (double) (input.data[0].y + input.data[1].y) / 2, 1);
        x8_pixel*=(1000*input.data[2].x);
        Mat calcWorld = invR * (invM * x8_pixel - T);//2D-3D变换
        calcWorld /= 1000;
        cout << calcWorld << endl;
        double x = calcWorld.at<double>(0, 0);
        double y = calcWorld.at<double>(1, 0);
        if(red_or_blue==0){
            y=field_width-y;
        }
        else
        {
            x=field_height-x;
        }
        double width = 0.5;
        double height = 0.5;
//        if (i == 20)i = 0;
//        point.x = 1 + i;
//        point.y = 1 + i;
//        worldPoint.data.push_back(point);
//        point.x = 2 + i;
//        point.y = 1 + i;
//        worldPoint.data.push_back(point);
//        point.x = 2 + i;
//        point.y = 2 + i;
//        worldPoint.data.push_back(point);
//        point.x = 1 + i;
//        point.y = 2 + i;
//        worldPoint.data.push_back(point);
//        i++;
//    worldPoint.z=calcWorld.at<double>(2,0);
        point.x = x - width / 2;
        point.y = y - height / 2;
        worldPoint.data.push_back(point);
        point.x = x + width / 2;
        point.y = y - height / 2;
        worldPoint.data.push_back(point);
        point.x = x + width / 2;
        point.y = y + height / 2;
        worldPoint.data.push_back(point);
        point.x = x - width / 2;
        point.y = y + height / 2;
        worldPoint.data.push_back(point);
        for (int i = 0; i < 4; i++) {
            worldPoint.data[i].x /= field_height;
            worldPoint.data[i].y /= field_width;
        }
        worldPoint.id = 0;
        worldPoint.color = string("red");
        worldPointPub.publish(worldPoint);
//        cout << worldPoint.data.size() << endl;
//        cout << worldPoint.data[0].x << "  " << worldPoint.data[0].y << endl;
        std::vector<radar_msgs::point>().swap(worldPoint.data);


    }
//    if(cout_flag==0) {
//        cout << calcWorld << endl;
//        cout_flag = 1;
//    }
}

void calibration(const radar_msgs::points &msg) {
    for (const auto &abc: msg.data) {
        imagePoints[abc.id] = cv::Point2d(abc.x, abc.y);
        imagePoints[abc.id].x *= imgCols;
        imagePoints[abc.id].y *= imgRows;
        cout << imagePoints[abc.id] << endl;
    }
    cout << "已经选出了4个点!下面进行SolvePnP求解外参矩阵。" << endl;
    cv::Mat abc;
    int suc = cv::solvePnPRansac(objectPoints, imagePoints, CamMatrix_, distCoeffs_, Rjacob, T, false, 100, 8.0, 0.99,
                                 abc, cv::SOLVEPNP_AP3P);
    // int suc=cv::solvePnP(objectPoints,imagePoints,CamMatrix_,distCoeffs_,Rjacob, T,false,cv::SOLVEPNP_EPNP);
    Rodrigues(Rjacob, R); //将R从雅可比形式转换为罗德里格斯形式,输出的R是3x3的一个矩阵。
    cout << "suc:" << suc << endl;
    cout << "旋转矩阵:" << R << endl;
    cout << "平移矩阵" << T << endl;
    calc_flag = 1;
}

void imageCB(
        const sensor_msgs::ImageConstPtr &msg
) {
    img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

    if (!img.empty()) {

        if (calc_flag)//如果已经计算出了R T
        {
//            //反投影：
//            cv::Mat invR;
//            cv::Mat invM;
//            invert(CamMatrix_, invM);
//            invert(R, invR);
//            cv::Mat x8_pixel;
//            x8_pixel = (cv::Mat_<double>(3, 1) << reprojectPoints[0].x, reprojectPoints[0].y, 1);
//            double s1 = 5207;//21961
//            cv::Mat calcWorld = invR * (invM * s1 * x8_pixel - T);//2D-3D变换
//            if (cout_flag == 0) {
//                cout << calcWorld << endl;
//                cout_flag = 1;
            project(objectPoints[0].x,objectPoints[0].y,objectPoints[0].z,img,R,T,CamMatrix_);
            project(objectPoints[1].x,objectPoints[1].y,objectPoints[1].z,img,R,T,CamMatrix_);
            project(objectPoints[2].x,objectPoints[2].y,objectPoints[2].z,img,R,T,CamMatrix_);
            project(objectPoints[3].x,objectPoints[3].y,objectPoints[3].z,img,R,T,CamMatrix_);
//             正投影：
//             cv::Mat x8_world,x8_img;
//             x8_world=(cv::Mat_<double>(3,1) <<9600,9600,0);//左上角点
//             x8_img=CamMatrix_*(R*x8_world+T);
//             x8_img/=x8_img.at<double>(2,0);
//             cv::circle(img,cv::Point2d(x8_img.at<double>(0,0),x8_img.at<double>(1,0)),5,cv::Scalar(0,0,255),-1,16,0);//画圆

        }
        cv::imshow(param_name + "project", img);
        char key = cv::waitKey(10);
        //按q拍照
        // if(key=='q')
        // {
        //   cout<<"save image!"<<endl;
        //   cv::imwrite("/home/chris/radar_station2022/img/MV_camera.jpg",img);
        // }
    }
}
