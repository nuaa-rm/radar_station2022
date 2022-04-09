#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <radar_msgs/relative_coordinate.h>

using namespace std;
cv::Mat img;
static cv::Point pre_pt = cv::Point(-1,-1);//初始坐标
static cv::Point cur_pt = cv::Point(-1,-1);//实时坐标
char temp[16];
void msgCallback(const radar_msgs::relative_coordinate::ConstPtr &msg)
{
  if(msg->id!=255)
  {
    //test_msgs::Test类型里的float32[]数据传到vector
    cv::Point2f car_point;
    car_point.x=msg->xaxis;
    car_point.y=msg->yaxis;
    cout<<car_point<<endl;
  }
  else if(msg->id==255)
  {
    cout<<"\n"<<endl;
  }

}

void onMouse(int event,int x,int y,int flags,void *ustc)//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
{
   
    if (event == cv::EVENT_LBUTTONDOWN&&!img.empty())//左键按下，读取初始坐标，并在图像上该点处划圆
    {
        sprintf(temp,"(%d,%d)",x,y);
        pre_pt = cv::Point(x,y);
        
    }
}

void imageCB(const sensor_msgs::ImageConstPtr& msg);


int main(int argc,char ** argv)
{
  /*相机标定 Camera Calibration*/
  


    ros::init(argc,argv,"point_subscribe");
    ros::NodeHandle n;
    // ros::Subscriber msg_sub = n.subscribe("relative_coordinate", 100, msgCallback);
    ros::Subscriber imageSub = n.subscribe("/MVCamera/image_raw", 1, &imageCB);
    ros::spin();
    return 0;
}


void imageCB(
    const sensor_msgs::ImageConstPtr& msg
    )
{
    img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    if(!img.empty())
    {
      cv::imshow("video",img);
      cv::setMouseCallback("video",onMouse,0);
      cv::putText(img,temp,pre_pt,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,0,255),1,8);//在窗口上显示坐标
      cv::circle(img,pre_pt,2,cv::Scalar(255,0,0,0),-1,16,0);//划圆
      cv::imshow("video",img);
      cv::waitKey(10);
    }



}