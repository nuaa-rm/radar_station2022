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
ros::Publisher worldPointPub;
cv::Mat img;
int cnt = 0;
vector<radar_msgs::distance_point> distance_points;
//世界坐标,取四个点用于solvePnP的objects_array
vector<cv::Point3f> objectPoints(5);
//像平面坐标,鼠标在图像上选点并用于solvePnP的imagePoints
vector<cv::Point2f> imagePoints(5);
vector<cv::Point2f> reprojectPoints(4);
vector<string> imagePoints_string(4);
//相机内参矩阵
// cv::Mat CamMatrix_ = (cv::Mat_<double>(3,3) << 919.1319, 0, 645.5211, 0, 919.7624, 539.8345, 0, 0, 1);
cv::Mat CamMatrix_ = (cv::Mat_<double>(3, 3) << 865.4454, 0, 307.764, 0, 865.9519, 221.537, 0, 0, 1);
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
cv::Mat distCoeffs_ = cv::Mat(1, 5, CV_64FC1, cv::Scalar::all(0));

void onMouse(int event, int x, int y, int flags, void *ustc);//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
void msgCallback(const radar_msgs::relative_coordinate::ConstPtr &msg);

void imageCB(const sensor_msgs::ImageConstPtr &msg);

void calibration(const radar_msgs::points &msg);//相机标定
void reproject(void);//反投影
void depthShow(Mat &input);//将深度图像归一化成灰度图并发布话题进行展示
void distPointCallback(const radar_msgs::distance_point &input);


int main(int argc, char **argv) {
    /*相机标定 Camera Calibration*/
    objectPoints[0].x = 0;
    objectPoints[0].y = 0;
    objectPoints[0].z = 1910;

    objectPoints[1].x = 14969;
    objectPoints[1].y = 15772;
    objectPoints[1].z = 930;

    // objectPoints[2].x=9600;
    // objectPoints[2].y=9600;
    // objectPoints[2].z=0;

    objectPoints[2].x = 19640;
    objectPoints[2].y = 12120;
    objectPoints[2].z = 0;

    // objectPoints[4].x=19725;
    // objectPoints[4].y=12120;
    // objectPoints[4].z=0;

    objectPoints[3].x = 19640;
    objectPoints[3].y = 11270;
    objectPoints[3].z = 0;

    objectPoints[4].x = 12200;
    objectPoints[4].y = 7200;
    objectPoints[4].z = 420;

    /*solvePnP求解相机外参矩阵*/


    ros::init(argc, argv, "point_subscribe");

    ros::NodeHandle n;

    // ros::Subscriber msg_sub = n.subscribe("relative_coordinate", 100, msgCallback);
//    imagesub = n.subscribe("/MVCamera/image_raw", 5, &imageCB);
//    ros::Subscriber imageSub = n.subscribe("/displayer/cameraOne/calibration", 5, &calibration);
    ros::Subscriber distPointSub = n.subscribe("/distance_point", 10, &distPointCallback);
    worldPointPub=n.advertise<radar_msgs::world_point>("/world_point",10);
    ros::spin();
    return 0;
}

void msgCallback(const radar_msgs::relative_coordinate::ConstPtr &msg) {
    if (msg->id != 255) {
        //test_msgs::Test类型里的float32[]数据传到vector
        cv::Point2f car_point;
        car_point.x = msg->xaxis;
        car_point.y = msg->yaxis;
        cout << car_point << endl;
    } else if (msg->id == 255) {
        cout << "\n" << endl;
    }

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

void distPointCallback(const radar_msgs::distance_point &input) {
    Mat invR;
    Mat invM;
    invert(CamMatrix_, invM);
    invert(R, invR);
    Mat x8_pixel;
    x8_pixel = (Mat_<double>(3, 1) << (double) input.point.x, (double) input.point.y, 1);
    Mat calcWorld = invR * (invM * input.depth * x8_pixel - T);//2D-3D变换

    radar_msgs::world_point worldPoint;
    worldPoint.id=input.point.id;
    worldPoint.x=calcWorld.at<double>(0,0);
    worldPoint.y=calcWorld.at<double>(1,0);
    worldPoint.z=calcWorld.at<double>(2,0);
    worldPointPub.publish(worldPoint);
    cout << calcWorld << endl;
//    if(cout_flag==0) {
//        cout << calcWorld << endl;
//        cout_flag = 1;
//    }
}

void imageCB(
        const sensor_msgs::ImageConstPtr &msg
) {
    img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

    if (!img.empty()) {
        //鼠标点击事件，记录点到的图像坐标
        cv::imshow("reproject", img);
        cv::setMouseCallback("reproject", onMouse, 0);
        auto textit = imagePoints_string.begin();
        for (auto it = reprojectPoints.begin(); it < reprojectPoints.end(); it++) {
            cv::putText(img, (*textit).c_str(), *it, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, 8);
            cv::circle(img, *it, 2, cv::Scalar(255, 255, 255), -1, 16, 0);//画圆
            textit++;
        }
        if (calc_flag && cnt)//如果已经计算出了R T
        {
            //反投影：
            cv::Mat invR;
            cv::Mat invM;
            invert(CamMatrix_, invM);
            invert(R, invR);
            cv::Mat x8_pixel;
            x8_pixel = (cv::Mat_<double>(3, 1) << reprojectPoints[0].x, reprojectPoints[0].y, 1);
            double s1 = 5207;//21961
            cv::Mat calcWorld = invR * (invM * s1 * x8_pixel - T);//2D-3D变换
            if (cout_flag == 0) {
                cout << calcWorld << endl;
                cout_flag = 1;


                //正投影：
                // cv::Mat x8_world,x8_img;
                // x8_world=(cv::Mat_<double>(3,1) <<9600,9600,0);//左上角点
                // x8_img=CamMatrix_*(R*x8_world+T);
                // x8_img/=x8_img.at<double>(2,0);
                // cv::circle(img,cv::Point2d(x8_img.at<double>(0,0),x8_img.at<double>(1,0)),5,cv::Scalar(0,0,255),-1,16,0);//画圆
            }
        }
        cv::imshow("reproject", img);
        char key = cv::waitKey(10);
        //按q拍照
        // if(key=='q')
        // {
        //   cout<<"save image!"<<endl;
        //   cv::imwrite("/home/chris/radar_station2022/img/MV_camera.jpg",img);
        // }
    }
}


void calibration(const radar_msgs::points &msg) {
    for (const auto &abc: msg.data) {
        imagePoints[abc.id] = cv::Point2d(abc.x, abc.y);
        imagePoints[abc.id].x *= 1080;
        imagePoints[abc.id].y *= 864;
        cout << imagePoints[abc.id] << endl;
        // cout<<cv::Point2d(abc.x,abc.y)<<endl;
    }
    // cout<<img.cols<<endl;
    // cout<<img.rows<<endl;
    cout << "已经选出了4个点!下面进行SolvePnP求解外参矩阵。" << endl;
    cv::Mat abc;
    int suc = cv::solvePnPRansac(objectPoints, imagePoints, CamMatrix_, distCoeffs_, Rjacob, T, false, 100, 8.0, 0.99,
                                 abc, cv::SOLVEPNP_P3P);
    // int suc=cv::solvePnP(objectPoints,imagePoints,CamMatrix_,distCoeffs_,Rjacob, T,false,cv::SOLVEPNP_EPNP);
    Rodrigues(Rjacob, R); //将R从雅可比形式转换为罗德里格斯形式,输出的R是3x3的一个矩阵。
    cout << "suc:" << suc << endl;
    cout << "旋转矩阵:" << R << endl;
    cout << "平移矩阵" << T << endl;
    calc_flag = 1;




    /*
  [69.9938, 345.6]
  [1023.94, 429.213]
  [679.43, 777.043]
  [419.628, 784.846]
  [131.963, 441.476]
  已经选出了4个点!下面进行SolvePnP求解外参矩阵。
  suc:1
  旋转矩阵:[-0.1939506224760869, 0.9810107994098256, 0.00098360684916865;
   -0.1559305285710967, -0.02983829646982272, -0.9873172470504694;
   -0.968539532647314, -0.1916441689827121, 0.1587566886598766]
  平移矩阵[-6831.473804735295;
   5409.432086526362;
   24360.80329266706]
  [17341.26296644072;
   12429.61885740408;
   150.919077897785]

    */



    // if(!img.empty())
    // {
    //   cv::imshow("video",img);
    //   cv::setMouseCallback("video",onMouse,0);
    // }

    /*
    Mat x8_world;
    x8_world=(Mat_<double>(3,1) << object_points_[0][8].x,object_points_[0][8].y,object_points_[0][8].z);//世界坐标
    Mat x8_pixel;
    x8_pixel=CamMatrix_*(R*x8_world+T);//像素坐标
    cout<<x8_pixel<<endl;
    double s =x8_pixel.at<double>(2,0);//深度信息，以mm为单位
    x8_pixel=x8_pixel/x8_pixel.at<double>(2,0);//除以深度信息
    cout<<x8_pixel<<endl;
    Mat invR;
    Mat invM;
    invert(CamMatrix_,invM);
    invert(R,invR);
    cout<<"CamMatrix_:\n"<<CamMatrix_<<endl;
    Mat calcWorld=invR*(invM*s*x8_pixel-T);//2D-3D变换
    cout<<"calcWorld:\n"<<calcWorld<<endl;
    */
}

// void reproject(cv::Point2d pixel_point)
// {

// }
