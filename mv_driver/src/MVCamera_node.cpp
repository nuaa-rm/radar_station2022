#include<iostream>
#include<vector>
#include <sstream>
using namespace std;

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "MVCamera.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include "video_saver.h"
#define FARCAM 0
#define CLOSECAM 1
using namespace std;
using namespace cv;

MVCamera *mv_driver=NULL;
Size dist_size=Size(640,512);

ros::Time imgTime;

class MVCamNode
{
public:
    ros::NodeHandle node_;
    int false_idx=0;
    int deviceID=0;
    int count=0;
    bool flag=true;
    // shared image message
    Mat rawImg;
    sensor_msgs::ImagePtr msg;
    image_transport::Publisher image_pub_;
    ros::Subscriber cfg_exp_sub;
    ros::Subscriber is_large_sub;
    ros::Subscriber is_rcd_sub;

    int image_width_, image_height_, framerate_, exposure_=2400, brightness_, contrast_, saturation_, sharpness_, focus_,
    white_balance_, gain_,fps_mode=1;
    bool large_resolution_=false,is_record_=false,autofocus_, autoexposure_=1, auto_white_balance_=1;
    string rcd_path_;
    VideoSaver saver;
    MVCamNode():
        node_()
    {
        image_transport::ImageTransport it(node_);
        cfg_exp_sub=node_.subscribe("/mv_param/exp_time",1,&MVCamNode::get_exp,this);
        //is_large_sub=node_.subscribe("/mv_param/is_large",1,&MVCamNode::get_is_large,this);  //if we want to use small resolution, comment this
        is_rcd_sub=node_.subscribe("/mv_param/is_record",1,&MVCamNode::get_is_rcd,this);
        ros::param::get("deviceID", deviceID);
        image_pub_ = it.advertise("image_raw", 1);

        node_.param("image_width", image_width_, 640);
        if(large_resolution_)
        {
            node_.param("image_height", image_height_, 512);
            node_.param("framerate", framerate_, 100);
        }
        else
        {
            node_.param("image_height", image_height_, 480);
            node_.param("framerate", framerate_, 30);


        }
        node_.param("/framerate", framerate_, 360);
        node_.getParam("/is_record", is_record_);
        string project_path(PROJECT_PATH);
        if(deviceID == FARCAM)
        {
            rcd_path_ = std::string(PROJECT_PATH) + "/sensor_far";
        }
        else if(deviceID == CLOSECAM)
        {
            rcd_path_ = std::string(PROJECT_PATH) + "/sensor_close";
        }
        node_.getParam("/fps_mode", fps_mode);
        node_.param("/exp_time", exposure_);
        //init camera param
        mv_driver=new MVCamera;

        mv_driver->Init(deviceID);
        mv_driver->SetExposureTime(autoexposure_, exposure_);
        mv_driver->SetLargeResolution(large_resolution_);
        mv_driver->Set_fps(fps_mode);
        mv_driver->Play();


    }
    ~MVCamNode()
    {
        mv_driver->Stop();
        mv_driver->Uninit();
    }
    ///
    /// \brief get_exp
    /// get exposure time
    /// \param exp_time
    ///
    void get_exp(const std_msgs::Int16ConstPtr &exp_time)
    {
        if(exposure_!=exp_time->data)
        {
            exposure_=exp_time->data;
            mv_driver->SetExposureTime(autoexposure_, exposure_);

        }
    }
    void get_is_large(const std_msgs::BoolConstPtr &is_large_resolution)
    {
        if(is_large_resolution->data!=large_resolution_) //dafu
        {
            // large_resolution_=is_large_resolution->data;
            // mv_driver->SetLargeResolution(large_resolution_);
            mv_driver->SetExposureTime(0, 3000);
        }else{
            mv_driver->SetExposureTime(0, 1500);
        }
    }
    void get_is_rcd(const std_msgs::BoolConstPtr &is_rcd)
    {
        if(is_record_!=is_rcd->data)
        {
            is_record_=is_rcd->data;
        }
    }
    string num2str(double i)

    {
        stringstream ss;
        ss << i;
        return ss.str();
    }
    ///
    /// \brief take_and_send_image
    /// use camera API in MVCamera.cpp
    /// \return
    ///
    bool take_and_send_image()
    {
        // grab the image

        mv_driver->GetFrame_B(rawImg,1);
        imgTime=ros::Time::now();
        if(rawImg.empty())
        {
            ROS_WARN("NO IMG GOT FROM MV");
            return false;
        }
        if(is_record_)
        {
            saver.write(rawImg,rcd_path_);
        }
        if(large_resolution_)
            resize(rawImg,rawImg,dist_size);
//        imshow("raw_img",rawImg);
//        int k = waitKey(30);
//        if(k == 'q' && !rawImg.empty())
//        {
//            ROS_INFO("Get %d far pictures!", count);
//            cout << rawImg.size().width << '\t' << rawImg.size().height <<endl;
//            stringstream ss;
//            ss<<"/home/chris/ws_livox/src/camera_lidar_calibration/data/photo/"<<count<<".bmp";
//            cv::imwrite(ss.str(),rawImg);
//            count++;
//        }
        std_msgs::Header imgHead;

        //DoveJH：用于在订阅者节点区分两个相机。
        if(deviceID == FARCAM)
        {
            imgHead.frame_id = "sensor_far";
//            cout<<"far:"<<rawImg.size()<<endl;
        }
        else if(deviceID == CLOSECAM)
        {
            imgHead.frame_id = "sensor_close";
//            cout<<"close:"<<rawImg.size()<<endl;
        }
        imgHead.stamp=imgTime;
        msg= cv_bridge::CvImage(imgHead, "bgr8", rawImg).toImageMsg();
        // publish the image
        image_pub_.publish(msg);

//        std::cout<<rawImg.size<<std::endl;
        return true;
    }

    bool spin()
    {
        ros::Rate loop_rate(this->framerate_);
        while (node_.ok())
        {
            imgTime=ros::Time::now();
            if (!mv_driver->stopped) {
                if (!take_and_send_image()) ROS_WARN("MVcamera did not respond in time.");
            }
            ros::spinOnce();
            loop_rate.sleep();

        }
        return true;
    }




};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"MVcamera_node");

    MVCamNode mv_node;


    mv_node.spin();
    return EXIT_SUCCESS;


}
