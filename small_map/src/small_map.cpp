#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <yolo/relative_coordinate.h>

using namespace std;
void msgCallback(const yolo::relative_coordinate::ConstPtr &msg)
{
  //test_msgs::Test类型里的float32[]数据传到vector
  cv::Point2f car_point ;
  car_point.x=msg->xaxis;
  car_point.y=msg->yaxis;
  cout<<car_point<<endl;
  

}

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"point_subscribe");
    ros::NodeHandle n;

    ros::Subscriber msg_sub = n.subscribe("relative_coordinate", 100, msgCallback);

    ros::spin();
    return 0;
}
