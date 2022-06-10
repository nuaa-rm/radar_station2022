//
// Created by dovejh on 2022/5/4.
//
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include "CRC8_CRC16.h"
#include "CRC.h"
#include "radar_msgs/points.h"
#include "radar_msgs/small_map.h"
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

struct car_point
{
    Point2f point;
    bool color; //红色为0 蓝色为1
};
struct frame_header//消息头
{
 uint8_t SOF = 0xA5;
 uint16_t data_length = 10;
 uint8_t seq;
 uint8_t crc;
} __attribute__((packed));
struct map_msg_data//小地图消息数据
{
    uint16_t target_robot_id;
    float target_position_x;
    float target_position_y;
} __attribute__((packed));
struct map_msg//小地图消息10HZ
{
    frame_header head;
    uint16_t cmd_id = 0x0305;
    map_msg_data data;
    uint16_t crc;
} __attribute__((packed));
struct interactive_with_robots_msgs_data
{
    uint16_t cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t number = 0;
} __attribute__((packed));
struct interactive_with_robots_msgs//目前是专门给工程写的
{
    frame_header head;
    uint16_t cmd_id = 0x0301;
    interactive_with_robots_msgs_data data;
    uint16_t crc;
} __attribute__((packed));//最大10HZ
struct game_status_msgs_data
{
    uint8_t game_type = 1;
    uint16_t game_progress = 0;
    uint16_t stage_remain_time = 0;
    uint64_t SyncTimeStamp = 0;

} __attribute__((packed));
struct game_status_msgs //1HZ
{
    frame_header head;
    uint16_t cmd_id = 0x0000;
    game_status_msgs_data data;
    uint16_t crc;
} __attribute__((packed));
class serial_port
{
public:
    serial::Serial ser;
    map_msg mapMsg;
    interactive_with_robots_msgs interactiveWithRobotsMsgs;
    game_status_msgs gameStatusMsgs;
    uint8_t receiveData[200];
    bool is_enemy_red = true;
    //vector<point>points;
    int serial_port_init()
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    serial_port()
    {
        serial_port_init();
    }
    bool sendMapMsgs(uint16_t id, float x, float y)
    {
        mapMsg.head.SOF = 0xA5;
        mapMsg.head.data_length = 10;
        mapMsg.head.seq = 1;
        mapMsg.head.crc = get_CRC8_check_sum((uint8_t*)&mapMsg, (sizeof(mapMsg.head) - sizeof(mapMsg.head.crc)), 0xff);
        mapMsg.cmd_id = 0x0305;
        mapMsg.data.target_position_x = x;
        mapMsg.data.target_position_y = y;
        mapMsg.data.target_robot_id = id;
        mapMsg.crc = get_CRC16_check_sum((uint8_t*)&mapMsg, (sizeof(mapMsg) - sizeof(mapMsg.crc)), 0xffff);

        ser.write((uint8_t* )&mapMsg, sizeof(map_msg));
        cout << "Send one map msg target_id = " << mapMsg.data.target_robot_id << " x = " << mapMsg.data.target_position_x << " y = " << mapMsg.data.target_position_y << endl;
        return true;
    }
    bool sendInteractiveMsgs(uint8_t number)//只适用于工程
    {
        interactiveWithRobotsMsgs.head.SOF = 0xA5;
        interactiveWithRobotsMsgs.head.data_length = 7;
        interactiveWithRobotsMsgs.head.seq = 1;
        interactiveWithRobotsMsgs.head.crc = get_CRC8_check_sum((uint8_t*)&interactiveWithRobotsMsgs, (sizeof(interactiveWithRobotsMsgs.head) - sizeof(interactiveWithRobotsMsgs.head.crc)), 0xff);
        interactiveWithRobotsMsgs.cmd_id = 0x0301;
        interactiveWithRobotsMsgs.data.cmd_id = 0x0200;
        if(is_enemy_red)
        {
            interactiveWithRobotsMsgs.data.sender_id = 109;
            interactiveWithRobotsMsgs.data.receiver_id = 102;
        }
        else
        {
            interactiveWithRobotsMsgs.data.sender_id = 9;
            interactiveWithRobotsMsgs.data.receiver_id = 2;
        }

        interactiveWithRobotsMsgs.data.number = number;
        interactiveWithRobotsMsgs.crc = get_CRC16_check_sum((uint8_t*)&interactiveWithRobotsMsgs, (sizeof(interactiveWithRobotsMsgs) - sizeof(interactiveWithRobotsMsgs.crc)), 0xffff);

        ser.write((uint8_t* )&interactiveWithRobotsMsgs, sizeof(interactiveWithRobotsMsgs));

        cout << "Send one interactive msg " << endl;
        return true;
    }
    bool receiveMsgs()
    {
        if(ser.available())
        {
            ser.read(receiveData, ser.available());
            gameStatusMsgs = *(game_status_msgs*)receiveData;
            if((gameStatusMsgs.head.crc == get_CRC8_check_sum((uint8_t*)&gameStatusMsgs, (sizeof(gameStatusMsgs.head) - sizeof(gameStatusMsgs.head.crc)), 0xff)) && (gameStatusMsgs.crc == get_CRC16_check_sum((uint8_t*)&gameStatusMsgs, (sizeof(gameStatusMsgs) - sizeof(gameStatusMsgs.crc)), 0xffff)))
            {

                //这里还没写完，需要新的消息。

                return true;
            }
        }
        return false;
    }

};
serial_port sp;
vector<car_point>worldPoints;
void worldPointsCallback(const radar_msgs::points& msg)
{

    for(int i = 0; i < msg.data.size(); i++)
    {
        if(msg.color == "red")
        {
            if(sp.is_enemy_red)
            {
                car_point carPoint;
                carPoint.color = 0;
                carPoint.point = Point(msg.data[i].x, msg.data[i].y);
                worldPoints.insert(worldPoints.begin(), carPoint);
            }
            else
            {
                car_point carPoint;
                carPoint.color = 0;
                carPoint.point = Point(msg.data[i].x, msg.data[i].y);
                worldPoints.push_back(carPoint);
            }
        }
        else if(msg.color == "blue")
        {
            if(sp.is_enemy_red)
            {
                car_point carPoint;
                carPoint.color = 1;
                carPoint.point = Point(msg.data[i].x, msg.data[i].y);
                worldPoints.push_back(carPoint);
            }
            else
            {
                car_point carPoint;
                carPoint.color = 1;
                carPoint.point = Point(msg.data[i].x, msg.data[i].y);
                worldPoints.insert(worldPoints.begin(), carPoint);
            }
        }
    }
}
int main (int argc, char** argv)
{
    //初始化节点
    ros::init(argc, argv, "serial_port_node");
    //声明节点句柄
    ros::NodeHandle nh;

    if(!sp.ser.isOpen())
    {
        ROS_ERROR_STREAM("Unable to open port, please check USB2TTL! ");
        return -1;
    }
    else
    {
        ROS_INFO_STREAM("Serial Port initialized! ");
    }
    ros::param::get("is_enemy_red", sp.is_enemy_red);
    ros::Subscriber worldPointSub = nh.subscribe("/world_point", 1, &worldPointsCallback);
    ros::Rate loop(10);
    ROS_INFO_STREAM("Looping! ");
    float x = 0, y = 0;
    int r = 1, b = 1;
    while(ros::ok())
    {
        if(!worldPoints.empty())
        {
            if(worldPoints[0].color)
            {
                sp.sendMapMsgs(100 + b, worldPoints[0].point.x, worldPoints[0].point.x);
                b++;
            }
            else
            {
                sp.sendMapMsgs(r, worldPoints[0].point.x, worldPoints[0].point.x);
                r++;
            }
            worldPoints.erase(worldPoints.begin());
        }
        else
        {
            r = 1;
            b = 1;
            ros::spinOnce();
        }

        //循环休眠
        loop.sleep();
    }
    return 0;
}