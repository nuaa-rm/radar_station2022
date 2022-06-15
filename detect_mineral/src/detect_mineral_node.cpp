//
// Created by dovejh on 2022/5/29.
//
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <radar_msgs/points.h>
#include "std_msgs/Int8.h"
using namespace std;
using namespace cv;

bool if_ready = true;
ros::Publisher mineralPub;
void mouse(int event, int x, int y, int flags, void*)
{
    if (event == EVENT_LBUTTONDOWN) //单击左键，输出坐标
    {
        cout << x << " , " << y << endl;
    }
}
double LineFitLeastSquares(vector<Point>points) //最小二乘法拟合直线
{
    if(points.size() <= 2)
    {
        return 1;
    }
    double A = 0.0;
    double B = 0.0;
    double C = 0.0;
    double D = 0.0;
    double E = 0.0;
    double F = 0.0;

    for (int i = 0; i < points.size(); i++)
    {
        A += points[i].x * points[i].x;
        B += points[i].x;
        C += points[i].x * points[i].y;
        D += points[i].y;
    }

    // 计算斜率a和截距b
    double a, b, temp = 0;
    if( temp = (points.size() * A - B * B) )// 判断分母不为0
    {
        a = (points.size() * C - B * D) / temp;
        b = (A * D - B * C) / temp;
    }
    else
    {
        a = 1;
        b = 0;
    }

    // 计算相关系数r
    double Xmean, Ymean;
    Xmean = B / points.size();
    Ymean = D / points.size();

    double tempSumXX = 0.0, tempSumYY = 0.0;
    for (int i = 0; i < points.size(); i++)
    {
        tempSumXX += (points[i].x - Xmean) * (points[i].x - Xmean);
        tempSumYY += (points[i].y - Ymean) * (points[i].y - Ymean);
        E += (points[i].x - Xmean) * (points[i].y - Ymean);
    }
    F = sqrt(tempSumXX) * sqrt(tempSumYY);

    double r;
    r = E / F;
    return r;
}
void imgCallback(const sensor_msgs::ImageConstPtr& msg)
{
    static bool if_temp_init = 0;
    static bool if_boundary_init = 0;
    static int boundary12 = 120, boundary23 = 210, boundary34 = 295, boundary45 = 385;
    static Mat temp = imread("/home/dovejh/project/radar_station/temp5.png");
    Mat roi;
    Mat imgs[3], imgBindary;
    static bool led_state[5][5] = {true};
    static bool last_led_state[5] = {true};
    static int send_state[5] = {0};
    Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    Rect rect(707, 401, 50, 19);
    if(if_ready)
    {
        if(!if_temp_init)
        {
            if(!temp.empty())
            {
                Mat result;
                matchTemplate(img, temp, result, TM_CCOEFF_NORMED);//模板匹配
                double maxVal, minVal;
                Point minLoc, maxLoc;
                //寻找匹配结果中的最大值和最小值以及坐标位置
                minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
                //绘制最佳匹配区域
                if(maxVal >= 0.5)
                {
                    rect = Rect(maxLoc.x, maxLoc.y, temp.cols, temp.rows);
                    ROS_WARN("Template matched successfully! Now ROI is %d, %d, %d, %d", rect.x, rect.y, rect.width, rect.height);
                    if_temp_init = true;
                }
            }
            else
            {
                ROS_WARN("Template load ERROR!!!");
                if_temp_init = true;
            }

        }

        roi = img(rect);
        resize(roi, roi, Size(500, 190));

        //三通道二值化
        split(roi, imgs);
        threshold(imgs[0], imgs[0], 200, 255, THRESH_BINARY);
        threshold(imgs[1], imgs[1], 200, 255, THRESH_BINARY);
        threshold(imgs[2], imgs[2], 200, 255, THRESH_BINARY);
        bitwise_and(imgs[0], imgs[1], imgs[1]);
        bitwise_and(imgs[1], imgs[2], imgBindary);

        vector<vector<Point>> contours;  //轮廓
        findContours(imgBindary, contours, RETR_LIST, CHAIN_APPROX_SIMPLE, Point());
        if(!if_boundary_init && contours.size() == 5)//最小二乘确定五个点的共线程度
        {
            vector<Point>points;
            int x[5];
            for(int n = 0; n < 5; n++)
            {
                Rect rect = boundingRect(contours[n]);
                points.push_back(Point(rect.x + 0.5 * rect.width, rect.y + 0.5 * rect.height));
                x[n] = rect.x + 0.5 * rect.width;
            }
            if(abs(LineFitLeastSquares(points)) >= 0.98)
            {
                sort(x, x + 5);
                boundary12 = (x[0] + x[1]) / 2;
                boundary23 = (x[1] + x[2]) / 2;
                boundary34 = (x[2] + x[3]) / 2;
                boundary45 = (x[3] + x[4]) / 2;
                ROS_WARN("Boundary has been changed! %d, %d, %d, %d", boundary12, boundary23, boundary34, boundary45);
                if_boundary_init = true;
            }

        }

        vector<Point>points;
        for(int n = 0; n < contours.size(); n++)
        {
            Rect rect = boundingRect(contours[n]);
            points.push_back(Point(rect.x + 0.5 * rect.width, rect.y + 0.5 * rect.height));
        }
        //cout << abs(LineFitLeastSquares(points)) << endl;
        if(abs(LineFitLeastSquares(points)) >= 0.98)
        {
            led_state[0][4] = 0;
            led_state[1][4] = 0;
            led_state[2][4] = 0;
            led_state[3][4] = 0;
            led_state[4][4] = 0;
            for (int n = 0; n < contours.size(); n++)
            {
                // 最大外接矩形
                Rect rect = boundingRect(contours[n]);
                rectangle(imgBindary, rect, Scalar(255, 0, 0), 2, 8, 0);
                if((rect.x + 0.5 * rect.width) < boundary12)
                {
                    led_state[0][4] = 1;
                }
                else if((rect.x + 0.5 * rect.width) < boundary23 && (rect.x + 0.5 * rect.width) >= boundary12)
                {
                    led_state[1][4] = 1;
                }
                else if((rect.x + 0.5 * rect.width) < boundary34 && (rect.x + 0.5 * rect.width) >= boundary23)
                {
                    led_state[2][4] = 1;
                }
                else if((rect.x + 0.5 * rect.width) < boundary45 && (rect.x + 0.5 * rect.width) >= boundary34)
                {
                    led_state[3][4] = 1;
                }
                else if((rect.x + 0.5 * rect.width) >= boundary45)
                {
                    led_state[4][4] = 1;
                }
            }
            for(int i = 0; i < 5; i++)
            {
                if((led_state[i][0] != last_led_state[i]) && led_state[i][0] == led_state[i][1] && led_state[i][0] == led_state[i][2] && led_state[i][0] == led_state[i][3] && led_state[i][0] == led_state[i][4])
                {
                    send_state[i] = 30;
                    last_led_state[i] = led_state[i][0];
                }
                if(send_state[i])
                {
                    std_msgs::Int8 mineralMsg;
                    mineralMsg.data = (i + 1);
                    mineralPub.publish(mineralMsg);
                    /*if(i == 0)
                    {
                        circle(imgBindary, Point(77, 122), 10, Scalar(0, 0, 0), 2);
                    }
                    else if(i == 1)
                    {
                        circle(imgBindary, Point(167, 105), 10, Scalar(0, 0, 0), 2);
                    }
                    else if(i == 2)
                    {
                        circle(imgBindary, Point(254, 92), 10, Scalar(0, 0, 0), 2);
                    }
                    else if(i == 3)
                    {
                        circle(imgBindary, Point(339, 77), 10, Scalar(0, 0, 0), 2);
                    }
                    else if(i == 4)
                    {
                        circle(imgBindary, Point(420, 62), 10, Scalar(0, 0, 0), 2);
                    }*/
                    cout << mineralMsg.data << endl;
                    send_state[i]--;
                }
            }
            /*for(int i = 0; i < 5; i++)
            {
                cout << send_state[i] << ' ';
            }
            cout << endl;*/
            for(int i = 0; i < 4; i++)
            {
                led_state[0][i] = led_state[0][i + 1];
                led_state[1][i] = led_state[1][i + 1];
                led_state[2][i] = led_state[2][i + 1];
                led_state[3][i] = led_state[3][i + 1];
                led_state[4][i] = led_state[4][i + 1];
            }
        }
        imshow("ROI", roi);
        imshow("temp", temp);
        imshow("bindary", imgBindary);
        waitKey(1);

    }

}
void paramCallback(const radar_msgs::pointsConstPtr& msg)
{
    if_ready = true;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_mineral_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/sensor_far/image_raw", 1, &imgCallback);
    ros::Subscriber paramSub = n.subscribe("/sensor_close/calibration", 1, &paramCallback);
    mineralPub = n.advertise<std_msgs::Int8>("/mineral", 0);

    ros::spin();
}