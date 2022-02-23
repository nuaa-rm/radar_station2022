//2021.12
//DoveJH

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
using namespace std;
using namespace cv;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

struct callback_args
{
    PointCloudT::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};
int mode = 0;//0：相机标定 1：点云获取 2：图片角点获取 3：联合标定
bool flag = true;
boost::mutex cloud_mutex;
vector<Point3f> cloudPoints;
vector<Point2f> imgPoints;
vector<Point3f> testPoints;
Mat img2;
Mat camera_matrix = Mat_<float>(3, 3);
Mat distortion_coefficient = Mat_<float>(5, 1);

bool getTestPoints()
{
    Point3f testPoint;
    testPoint.x = (cloudPoints[0].x + cloudPoints[1].x) / 2;
    testPoint.y = (cloudPoints[0].y + cloudPoints[1].y) / 2;
    testPoint.z = (cloudPoints[0].z + cloudPoints[1].z) / 2;
    testPoints.push_back(testPoint);

    testPoint.x = (cloudPoints[2].x + cloudPoints[1].x) / 2;
    testPoint.y = (cloudPoints[2].y + cloudPoints[1].y) / 2;
    testPoint.z = (cloudPoints[2].z + cloudPoints[1].z) / 2;
    testPoints.push_back(testPoint);

    testPoint.x = (cloudPoints[2].x + cloudPoints[3].x) / 2;
    testPoint.y = (cloudPoints[2].y + cloudPoints[3].y) / 2;
    testPoint.z = (cloudPoints[2].z + cloudPoints[3].z) / 2;
    testPoints.push_back(testPoint);

    testPoint.x = (cloudPoints[0].x + cloudPoints[3].x) / 2;
    testPoint.y = (cloudPoints[0].y + cloudPoints[3].y) / 2;
    testPoint.z = (cloudPoints[0].z + cloudPoints[3].z) / 2;
    testPoints.push_back(testPoint);
}
void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
    struct callback_args* data = (struct callback_args *)args;
    if (event.getPointIndex() == -1)
        return;
    PointT current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);

    //TODO
    data->clicked_points_3d->clear();//将上次选的点清空

    data->clicked_points_3d->points.push_back(current_point);//添加新选择的点
    // Draw clicked points in red:将选中点用红色标记
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
    std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;

}

void mouse(int event, int x, int y, int flags, void*)
{
    Point2f imgPoint(x, y);
    if (event == EVENT_LBUTTONDOWN) //单击左键，输出坐标
    {
        circle(img2, imgPoint, 5, Scalar(0, 0, 255), 2);
        ROS_INFO("%d, %d", x, y);
    }
    imshow("image", img2);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    static vector<Mat> imgs;
    static int count = 0;
    if(mode == 0)
    {
        if(flag)
        {
            imshow("realsense", img);
            int k = waitKey(30);
            if(k == 'p' && !img.empty())
            {
                imgs.push_back(img);
                ROS_INFO("Get %d far pictures!", count);
                cout << img.size().width << '\t' << img.size().height <<endl;
                count++;
            }
            if(count >= 5)
            {
                flag = false;
            }
        }
        if(!flag)
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
    else if(mode == 2)
    {
        imshow("realsense", img);
        int k = waitKey(30);
        if(k == 'p' && !img.empty())
        {
            flag = false;
        }
        if(!flag)
        {
            img.copyTo(img2);
            imshow("image", img2);
            setMouseCallback("image", mouse, 0); //鼠标影响
            waitKey(0);
        }
    }
    else if(mode == 3)
    {
        imshow("realsense", img);
        int k = waitKey(30);
        if(k == 'p' && !img.empty())
        {
            flag = false;
        }
        if(!flag)
        {
            img.copyTo(img2);

            Mat rvec, tvec; //旋转向量和平移向量
            solvePnPRansac(cloudPoints, imgPoints, camera_matrix, distortion_coefficient, rvec, tvec, SOLVEPNP_EPNP);
            Mat RRansac;

            Rodrigues(rvec, RRansac);
            cout << "旋转向量转换成旋转矩阵：" << endl << RRansac << endl;
            cout << "平移向量："  <<endl << tvec << endl;

            //测试
            getTestPoints();
            vector<Point2f> imagePointsForTest;
            projectPoints(testPoints, rvec, tvec, camera_matrix, distortion_coefficient, imagePointsForTest);
            circle(img2, imagePointsForTest[0], 5, Scalar(0, 0, 255), 2);
            circle(img2, imagePointsForTest[1], 5, Scalar(0, 0, 255), 2);
            circle(img2, imagePointsForTest[2], 5, Scalar(0, 0, 255), 2);
            circle(img2, imagePointsForTest[3], 5, Scalar(0, 0, 255), 2);

            circle(img2, imgPoints[0], 5, Scalar(0, 0, 255), 2);
            circle(img2, imgPoints[1], 5, Scalar(0, 0, 255), 2);
            circle(img2, imgPoints[2], 5, Scalar(0, 0, 255), 2);
            circle(img2, imgPoints[3], 5, Scalar(0, 0, 255), 2);

            imshow("image", img2);

            waitKey(0);
        }
    }
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("realtime pcl"));
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;    //原始的点云的数据格式
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    // 转化为PCL中的点云的数据格式
    pcl_conversions::toPCL(*input, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1;
    cloud1.reset (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input, *cloud1);


    /*if(pointFlag)
    {
        viewer->removeAllPointClouds();  // 移除当前所有点云
        viewer->addPointCloud(cloud1, "realtime pcl");
        viewer->updatePointCloud(cloud1, "realtime pcl");
        viewer->spinOnce(0.001);
    }*/
    std::cout << cloud1->points.size() << std::endl;
    cloud_mutex.lock();
    viewer->addPointCloud(cloud1, "realtime pcl");
    struct callback_args cb_args;
    PointCloudT::Ptr clicked_points_3d(new PointCloudT);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
    //注册屏幕选点事件
    viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
    //Shfit+鼠标左键选择点
    std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

    //摁“Q”键结束
    viewer->spin();
    std::cout << "done." << std::endl;
    flag = false;
    //释放互斥体
    cloud_mutex.unlock();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);   //100??
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibrateCamera_node");
    ros::NodeHandle n;

    ros::param::get("/mode", mode);
    cout << "Mode lode done!" << mode << endl;

    ros::param::get("/camera_matrix/zerozero", camera_matrix.at<float>(0, 0));
    ros::param::get("/camera_matrix/zerotwo", camera_matrix.at<float>(0, 2));
    ros::param::get("/camera_matrix/oneone", camera_matrix.at<float>(1, 1));
    ros::param::get("/camera_matrix/onetwo", camera_matrix.at<float>(1, 2));
    ros::param::get("/camera_matrix/twotwo", camera_matrix.at<float>(2, 2));
    camera_matrix.at<float>(0, 1) = 0;
    camera_matrix.at<float>(1, 0) = 0;
    camera_matrix.at<float>(2, 0) = 0;
    camera_matrix.at<float>(2, 1) = 0;

    ros::param::get("/distortion_coefficient/zero", distortion_coefficient.at<float>(0, 0));
    ros::param::get("/distortion_coefficient/one", distortion_coefficient.at<float>(0, 1));
    ros::param::get("/distortion_coefficient/two", distortion_coefficient.at<float>(0, 2));
    ros::param::get("/distortion_coefficient/three", distortion_coefficient.at<float>(0, 3));
    ros::param::get("/distortion_coefficient/four", distortion_coefficient.at<float>(0, 4));

    cout << camera_matrix <<endl;
    cout << distortion_coefficient << endl;

    Point3f cloudPoint;
    ros::param::get("/cloue_points/one/x", cloudPoint.x);
    ros::param::get("/cloud_points/one/y", cloudPoint.y);
    ros::param::get("/cloud_points/one/z", cloudPoint.z);
    cloudPoints.push_back(cloudPoint);
    ros::param::get("/cloud_points/two/x", cloudPoint.x);
    ros::param::get("/cloud_points/two/y", cloudPoint.y);
    ros::param::get("/cloud_points/two/z", cloudPoint.z);
    cloudPoints.push_back(cloudPoint);
    ros::param::get("/cloud_points/three/x", cloudPoint.x);
    ros::param::get("/cloud_points/three/y", cloudPoint.y);
    ros::param::get("/cloud_points/three/z", cloudPoint.z);
    cloudPoints.push_back(cloudPoint);
    ros::param::get("/cloud_points/four/x", cloudPoint.x);
    ros::param::get("/cloud_points/four/y", cloudPoint.y);
    ros::param::get("/cloud_points/four/z", cloudPoint.z);
    cloudPoints.push_back(cloudPoint);
    cout << "Cloud points load done!" << endl;

    /*Point3f cloudPoint;
    cloudPoint.x = -15.3463;
    cloudPoint.y = 1.55666;
    cloudPoint.z = 53.7;
    cloudPoints.push_back(cloudPoint);
    cloudPoint.x = 12.6302;
    cloudPoint.y = 0.818345;
    cloudPoint.z = 55.2;
    cloudPoints.push_back(cloudPoint);
    cloudPoint.x = 13.3123;
    cloudPoint.y = 18.4942;
    cloudPoint.z = 49.5;
    cloudPoints.push_back(cloudPoint);
    cloudPoint.x = -14.7457;
    cloudPoint.y = 18.1974;
    cloudPoint.z = 48.4;
    cloudPoints.push_back(cloudPoint);*/

    Point imgPoint;
    ros::param::get("/img_points/one/x", imgPoint.x);
    ros::param::get("/img_points/one/y", imgPoint.y);
    imgPoints.push_back(imgPoint);
    ros::param::get("/img_points/two/x", imgPoint.x);
    ros::param::get("/img_points/two/y", imgPoint.y);
    imgPoints.push_back(imgPoint);
    ros::param::get("/img_points/three/x", imgPoint.x);
    ros::param::get("/img_points/three/y", imgPoint.y);
    imgPoints.push_back(imgPoint);
    ros::param::get("/img_points/four/x", imgPoint.x);
    ros::param::get("/img_points/four/y", imgPoint.y);
    imgPoints.push_back(imgPoint);
    cout << "Image points load done!" << endl;

    /*imgPoint.x = 426;
    imgPoint.y = 385;
    imgPoints.push_back(imgPoint);
    imgPoint.x = 885;
    imgPoint.y = 379;
    imgPoints.push_back(imgPoint);
    imgPoint.x = 920;
    imgPoint.y = 706;
    imgPoints.push_back(imgPoint);
    imgPoint.x = 406;
    imgPoint.y = 717;
    imgPoints.push_back(imgPoint);*/
    ros::Subscriber sub;
    if(mode == 1)//点云获取
    {
        sub = n.subscribe ("/camera/depth/color/points", 1, &pointCloudCallback);
    }
    else
    {
        sub = n.subscribe("/camera/color/image_raw", 1, &imageCallback);
    }
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    //ros::spin();
    return 0;
}
