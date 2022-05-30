//
// Created by dovejh on 2022/5/29.
//
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

Mat img = imread("/home/dovejh/project/radar_station/train/images/158.jpg");
Mat roi;
Mat imgs[3], imgBindary, img1 = Mat::ones(img.rows, img.cols, CV_8UC1),
img2 = Mat::ones(img.rows, img.cols, CV_8UC1),
img3 = Mat::ones(img.rows, img.cols, CV_8UC1),
img4 = Mat::ones(img.rows, img.cols, CV_8UC1),
img5 = Mat::ones(img.rows, img.cols, CV_8UC1),
img6 = Mat::ones(img.rows, img.cols, CV_8UC1);
int maxR = 255, maxG = 255, maxB = 255, minR = 0, minG = 0, minB = 0;
void callBackMaxR(int, void*)
{
    threshold(imgs[0], img1, maxR, 255, THRESH_BINARY_INV);
    //bitwise_and(img1, img2, imgs[0]);
    imshow("R1", img1);

}
void callBackMaxG(int, void*)
{
    threshold(imgs[1], img3, maxG, 255, THRESH_BINARY_INV);
    //bitwise_and(img1, img2, imgs[0]);
    imshow("G1", img3);
}
void callBackMaxB(int, void*)
{
    threshold(imgs[2], img5, maxB, 255, THRESH_BINARY_INV);
    //bitwise_and(img1, img2, imgs[0]);
    imshow("B1", img5);
}
void callBackMinR(int, void*)
{
    threshold(imgs[0], img2, minR, 255, THRESH_BINARY);
    //bitwise_and(img1, img2, imgs[0]);
    imshow("R2", img2);
}
void callBackMinG(int, void*)
{
    threshold(imgs[1], img4, minG, 255, THRESH_BINARY);
    //bitwise_and(img1, img2, imgs[0]);
    imshow("G2", img4);
}
void callBackMinB(int, void*)
{
    threshold(imgs[2], img6, minB, 255, THRESH_BINARY);
    //bitwise_and(img1, img2, imgs[0]);
    imshow("B2", img6);
}
void mouse(int event, int x, int y, int flags, void*)
{
    if (event == EVENT_LBUTTONDOWN) //单击左键，输出坐标
    {
        cout << x << " , " << y << endl;
    }
}
void imgCallback(const sensor_msgs::ImageConstPtr& msg)
{
    Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    Rect rect(707, 401, 50, 19);
    roi = img(rect);
    resize(roi, roi, Size(500, 190));

    split(roi, imgs);
    threshold(imgs[0], imgs[0], 200, 255, THRESH_BINARY);
    threshold(imgs[1], imgs[1], 200, 255, THRESH_BINARY);
    threshold(imgs[2], imgs[2], 200, 255, THRESH_BINARY);
    bitwise_and(imgs[0], imgs[1], imgs[1]);
    bitwise_and(imgs[1], imgs[2], imgBindary);
    imshow("bindary", imgBindary);
    waitKey(1);

}
void test()
{
    Rect rect(707, 401, 50, 19);
    roi = img(rect);
    resize(roi, roi, Size(500, 190));

    split(roi, imgs);
    /*namedWindow("roi");
    imshow("roi", roi);
    createTrackbar("maxR", "roi", &maxR, 255, callBackMaxR, 0);
    createTrackbar("minR", "roi", &minR, 255, callBackMinR, 0);
    createTrackbar("maxG", "roi", &maxG, 255, callBackMaxG, 0);
    createTrackbar("minG", "roi", &minG, 255, callBackMinG, 0);
    createTrackbar("maxB", "roi", &maxB, 255, callBackMaxB, 0);
    createTrackbar("minB", "roi", &minB, 255, callBackMinB, 0);*/
    //imshow("bindary")
    threshold(imgs[0], imgs[0], 200, 255, THRESH_BINARY);
    threshold(imgs[1], imgs[1], 200, 255, THRESH_BINARY);
    threshold(imgs[2], imgs[2], 200, 255, THRESH_BINARY);
    bitwise_and(imgs[0], imgs[1], imgs[1]);
    bitwise_and(imgs[1], imgs[2], imgBindary);

    //形态学运算

    vector<vector<Point>> contours;  //轮廓
    findContours(imgBindary, contours, RETR_LIST, CHAIN_APPROX_SIMPLE, Point());


    //共线筛选


    imshow("bindary", imgBindary);
    waitKey();
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_mineral_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/camera/color/image_raw", 1, &imgCallback);
    test();
    ros::spin();
}