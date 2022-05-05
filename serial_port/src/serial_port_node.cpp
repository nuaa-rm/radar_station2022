//
// Created by dovejh on 2022/5/4.
//
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include "CRC8_CRC16.h"
#include "CRC.h"
#include "radar_msgs/points.h"

using namespace std;

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
struct map_msg//小地图消息
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

} __attribute__((packed));
struct interactive_with_robots_msgs
{
    frame_header head;
    uint16_t cmd_id = 0x0301;
    interactive_with_robots_msgs_data data;
    uint16_t crc;
} __attribute__((packed));
class serial_port
{
public:
    serial::Serial ser;
    map_msg mapMsg;
    interactive_with_robots_msgs interactiveWithRobotsMsgs;
    void serial_port_init()
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
        mapMsg.head.seq = 0;
        mapMsg.head.crc = get_CRC8_check_sum((uint8_t*)&mapMsg.head, (sizeof(mapMsg.head) - sizeof(mapMsg.head.crc)), 0xff);
        mapMsg.cmd_id = 0x0305;
        mapMsg.data.target_position_x = x;
        mapMsg.data.target_position_y = y;
        mapMsg.data.target_robot_id = id;
        mapMsg.crc = get_CRC16_check_sum((uint8_t*)&mapMsg, (sizeof(mapMsg) - sizeof(mapMsg.crc)), 0xffff);
        string msg;
        for(int i = 0; i < sizeof(mapMsg); i++)
        {
            msg += ((char*)&mapMsg + i);
        }
        ser.write(msg);
        cout << "Send one map msg " << endl;
        return true;
    }
    bool sendInteractiveMsgs()
    {
        interactiveWithRobotsMsgs.head.SOF = 0xA5;

        string msg;
        for(int i = 0; i < sizeof(interactiveWithRobotsMsgs); i++)
        {
            msg += ((char*)&interactiveWithRobotsMsgs + i);
        }
        ser.write(msg);
        cout << "Send one interactive msg " << endl;
        return true;
    }
    bool receiveMsgs()
    {
        return true;
    }

};

int main (int argc, char** argv)
{
    //初始化节点
    ros::init(argc, argv, "serial_port_node");
    //声明节点句柄
    ros::NodeHandle nh;


    serial_port sp;
    if(!sp.ser.isOpen())
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    else
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }


    ros::Rate loop(10);
    while(ros::ok())
    {
        for(int x = 0; x < 28; x++)
        {
            for(int y = 0; y < 15; y++)
            {
                sp.sendMapMsgs(6, (float)x, (float)y);
            }
        }
        //循环休眠
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}