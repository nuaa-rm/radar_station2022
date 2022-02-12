//2021.12
//DoveJH

#include <opencv2/opencv.hpp>
#include <fstream> 
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>

using namespace std;
using namespace cv;

bool ifcalibrate = true;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    static vector<Mat> imgs;
    static int count = 0;
    if(ifcalibrate)
    {
        imshow("realsense", img);
        int k = waitKey(30);
        if(k == 'r' && !img.empty())
        {
            imgs.push_back(img);
            ROS_INFO("Get %d far pictures!", count);
			cout << img.size().width << '\t' << img.size().height <<endl;
            count++;
        }
        if(count >= 5)
        {
            ifcalibrate = false;
        }

        if(!ifcalibrate)
        {
            Size board_size = Size(8, 5);  //方格标定板内角点数目（行，列）
	        vector<vector<Point2f>> imgsPoints;
	        for (int i = 0; i < imgs.size(); i++)
	        {
		        Mat img1 = imgs[i];
		        Mat gray1;
		        cvtColor(img1, gray1, COLOR_BGR2GRAY);
		        vector<Point2f> img1_points;
		        findChessboardCorners(gray1, board_size, img1_points);  //计算方格标定板角点
		        find4QuadCornerSubpix(gray1, img1_points, Size(3, 3));  //细化方格标定板角点坐标
                drawChessboardCorners(img1, board_size, img1_points, true);
                imshow("chessboard", img1);
                waitKey(0);
		        imgsPoints.push_back(img1_points);
        	}

        	//生成棋盘格每个内角点的空间三维坐标
        	Size squareSize = Size2f (3, 3);  //棋盘格每个方格的真实尺寸
        	vector<vector<Point3f>> objectPoints;
	        for (int i = 0; i < imgsPoints.size(); i++)
	        {
		        vector<Point3f> tempPointSet;
		        for (int j = 0; j < board_size.height; j++)
		        {
			        for (int k = 0; k < board_size.width; k++)
			        {
				        Point3f realPoint;
				        // 假设标定板为世界坐标系的z平面，即z=0
				        realPoint.x = j*squareSize.width;
				        realPoint.y = k*squareSize.height;
				        realPoint.z = 0;
				        tempPointSet.push_back(realPoint);
			        }
		        }
		        objectPoints.push_back(tempPointSet);
	        }

	        /* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
	        vector<int> point_number;
	        for (int i = 0; i<imgsPoints.size(); i++)
	        {
		        point_number.push_back(board_size.width*board_size.height);
	        }

	        //图像尺寸
	        Size imageSize;
	        imageSize.width = imgs[0].cols;
	        imageSize.height = imgs[0].rows;

	        Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));  //摄像机内参数矩阵
	        Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));  //摄像机的5个畸变系数：k1,k2,p1,p2,k3
	        vector<Mat> rvecs;  //每幅图像的旋转向量
	        vector<Mat> tvecs;  //每张图像的平移量
	        calibrateCamera(objectPoints, imgsPoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, 0);
            for(int i = 0; i < tvecs.size(); i++)
            {
                cout << tvecs[i];
            }
	        cout << "相机的内参矩阵=" << endl << cameraMatrix << endl;
	        cout << "相机畸变系数" <<endl << distCoeffs << endl;
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibrateCamera_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/camera/color/image_raw", 1, &imageCallback);
    ros::spin();
    return 0;
}
