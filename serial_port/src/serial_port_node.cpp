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
#include "std_msgs/Int8.h"
using namespace std;
using namespace cv;

struct frame_header//消息头
{
    uint8_t SOF = 0xA5;//固定值
    uint16_t data_length = 10;//data的长度
    uint8_t seq; //包序号
    uint8_t crc; //帧头crc8
} __attribute__((packed));

struct map_data//小地图消息数据 10hz 发送
{
    uint16_t target_robot_id;
    float target_position_x;
    float target_position_y;
} __attribute__((packed));
struct map_msg
{
    frame_header head;
    uint16_t cmd_id = 0x0305;
    map_data data;
    uint16_t crc;
} __attribute__((packed));

struct robot_interactive_data//最大10HZ 发送和接收
{
    uint16_t cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t content[113];
} __attribute__((packed));
struct robot_interactive_msgs
{
    frame_header head;
    uint16_t cmd_id = 0x0301;
    robot_interactive_data data;
    uint16_t crc;
} __attribute__((packed));

struct robot_interactive_control_data //30HZ 发送和接收
{
    uint8_t content[30];
} __attribute__((packed));
struct robot_interactive_control_msgs
{
    frame_header head;
    uint16_t cmd_id = 0x0302;
    robot_interactive_control_data data;
    uint16_t crc;
} __attribute__((packed));

struct robot_health_data //1hz 接收
{
    uint16_t red_1_robot_HP = 0;
    uint16_t red_2_robot_HP = 0;
    uint16_t red_3_robot_HP = 0;
    uint16_t red_4_robot_HP = 0;
    uint16_t red_5_robot_HP = 0;
    uint16_t red_7_robot_HP = 0;
    uint16_t red_outpose_HP = 0;//前哨站
    uint16_t red_base_HP = 0;//基地

    uint16_t blue_1_robot_HP = 0;
    uint16_t blue_2_robot_HP = 0;
    uint16_t blue_3_robot_HP = 0;
    uint16_t blue_4_robot_HP = 0;
    uint16_t blue_5_robot_HP = 0;
    uint16_t blue_7_robot_HP = 0;
    uint16_t blue_outpose_HP = 0;//前哨站
    uint16_t blue_base_HP = 0;//基地
} __attribute__((packed));
struct robot_health_msgs //1HZ
{
    frame_header head;
    uint16_t cmd_id = 0x0003;
    robot_health_data data;
    uint16_t crc;
} __attribute__((packed));

struct game_status_data //1hz 接收
{
    uint8_t game_type : 4; //1：机架大师赛 2：单项赛 3：人工智能挑战赛 4：联盟赛3v3 5：联盟赛1v1
    uint8_t game_progress : 4; //0：未开始比赛 1：准备阶段 2：自检阶段 3：5s倒计时 4：对战中 5：比赛结算中
    uint16_t stage_remain_time = 0; //当前阶段剩余时间，单位s
    uint64_t SyncTimeStamp = 0; //机器人接收到该指令的精确Unix时间,当机载端收到有效的NTP服务器授时后生效

} __attribute__((packed));
struct game_status_msgs
{
    frame_header head;
    uint16_t cmd_id = 0x0001;
    game_status_data data;
    uint16_t crc;
} __attribute__((packed));

struct game_result_data//比赛结束发送 接收
{
    uint8_t winner; //0平局 1红方胜利 2蓝方胜利
} __attribute__((packed));
struct game_result_msg
{
    frame_header head;
    uint16_t cmd_id = 0x0002;
    game_result_data data;
    uint16_t crc;
} __attribute__((packed));

struct site_event_data //1hz 接收
{
    uint32_t event_type;
    //bit 0:己方补给站 1 号补血点占领状态 1 为已占领;
    //bit 1:己方补给站 2 号补血点占领状态 1 为已占领;
    //bit 2:己方补给站 3 号补血点占领状态 1 为已占领;
    //bit 3-5:己方能量机关状态:
    // bit 3 为打击点占领状态,1 为占领;
    // bit 4 为小能量机关激活状态,1 为已激活;
    // bit 5 为大能量机关激活状态,1 为已激活;
    //bit 6:己方侧 R2/B2 环形高地占领状态 1 为已占领;
    //bit 7:己方侧 R3/B3 梯形高地占领状态 1 为已占领;
    //bit 8:己方侧 R4/B4 梯形高地占领状态 1 为已占领;
    //bit 9:己方基地护盾状态: 1 为基地有虚拟护盾血量; 0 为基地无虚拟护盾血量;
    //bit 10:己方前哨战状态: 1 为前哨战存活; 0 为前哨战被击毁;
    //bit 10 -31: 保留
} __attribute__((packed));
struct site_event_msgs
{
    frame_header head;
    uint16_t cmd_id = 0x0101;
    site_event_data data;
    uint16_t crc;
} __attribute__((packed));

struct supply_projectile_action_data //触发时发送 接收
{
    uint8_t supply_projectile_id; //补给站口ID 1一号补给口 2二号补给口
    uint8_t supply_robot_id; //补弹机器人ID
    uint8_t supply_projectile_step; //出弹口开闭状态 0关闭 1子弹准备中 2子弹下落
    uint8_t supply_projectile_num; //补单数量 50:50颗子弹 100:100颗子弹 150...... 200......
} __attribute__((packed));
struct supply_projectile_action_msg
{
    frame_header head;
    uint16_t cmd_id = 0x0102;
    supply_projectile_action_data data;
    uint16_t crc;
} __attribute__((packed));

struct referee_warning_data //触发时发送 接收
{
    uint8_t level; //1黄牌 2红牌 3判负
    uint8_t foul_robot_id; //犯规机器人ID 判负时为0
} __attribute__((packed));
struct referee_warning_msg
{
    frame_header head;
    uint16_t cmd_id = 0x0104;
    referee_warning_data data;
    uint16_t crc;
} __attribute__((packed));

struct dart_remaining_time_data //1hz 接收
{
    uint8_t dart_remaining_time; //15s倒计时
} __attribute__((packed));
struct dart_remaining_time_msg
{
    frame_header head;
    uint16_t cmd_id = 0x0105;
    dart_remaining_time_data data;
    uint16_t crc;
} __attribute__((packed));

struct car_point
{
    uint16_t id;
    Point2f point;
    bool color; //红色为0 蓝色为1
};

class serial_port
{
public:
    serial::Serial ser;
    map_msg mapMsg;
    robot_interactive_msgs robotInteractiveMsgs;
    robot_interactive_control_msgs robotInteractiveControlMsgs;
    robot_health_msgs robotHealthMsgs;
    game_result_msg gameResultMsg;
    site_event_msgs siteEventMsgs;
    supply_projectile_action_msg supplyProjectileActionMsg;
    referee_warning_msg refereeWarningMsg;
    dart_remaining_time_msg dartRemainingTimeMsg;
    game_status_msgs gameStatusMsgs;
    uint8_t receiveData[1024];
    bool is_enemy_red = false;
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
    bool sendInteractiveMsgs(uint8_t *content, uint16_t receiver_id)//接受者ID以红方为准
    {
        //构造头
        robotInteractiveMsgs.head.SOF = 0xA5;
        robotInteractiveMsgs.head.data_length = sizeof(robot_health_data);
        robotInteractiveMsgs.head.seq = 1;
        robotInteractiveMsgs.head.crc = get_CRC8_check_sum((uint8_t*)&robotInteractiveMsgs, (sizeof(robotInteractiveMsgs.head) - sizeof(robotInteractiveMsgs.head.crc)), 0xff);
        robotInteractiveMsgs.cmd_id = 0x0301;
        if(is_enemy_red)
        {
            robotInteractiveMsgs.data.sender_id = 109;
            robotInteractiveMsgs.data.receiver_id = 100 + receiver_id;
        }
        else
        {
            robotInteractiveMsgs.data.sender_id = 9;
            robotInteractiveMsgs.data.receiver_id = 100 + receiver_id;
        }
        robotInteractiveMsgs.crc = get_CRC16_check_sum((uint8_t*)&robotInteractiveMsgs, (sizeof(robotInteractiveMsgs) - sizeof(robotInteractiveMsgs.crc)), 0xffff);
        ser.write((uint8_t* )&robotInteractiveMsgs, sizeof(robotInteractiveMsgs));
        cout << "Send one interactive msg " << endl;
        return true;
    }
    bool receiveMsgs()
    {
        if(ser.available())
        {
            ser.read(receiveData, ser.available());
            gameStatusMsgs = (*(game_status_msgs*)receiveData);
            dartRemainingTimeMsg = (*(dart_remaining_time_msg*)receiveData);
            robotHealthMsgs = (*(robot_health_msgs*)receiveData);
            gameResultMsg = (*(game_result_msg*)receiveData);
            siteEventMsgs = (*(site_event_msgs*)receiveData);
            supplyProjectileActionMsg = (*(supply_projectile_action_msg*)receiveData);
            refereeWarningMsg = (*(referee_warning_msg*)receiveData);
            dartRemainingTimeMsg = (*(dart_remaining_time_msg*)receiveData);
            if((gameStatusMsgs.head.crc == get_CRC8_check_sum((uint8_t*)&gameStatusMsgs, (sizeof(gameStatusMsgs.head) - sizeof(gameStatusMsgs.head.crc)), 0xff)) && (gameStatusMsgs.crc == get_CRC16_check_sum((uint8_t*)&gameStatusMsgs, (sizeof(gameStatusMsgs) - sizeof(gameStatusMsgs.crc)), 0xffff)))
            {
                cout << gameStatusMsgs.data.game_progress << endl;
                //这里还没写完，需要新的消息。
            }
            if((dartRemainingTimeMsg.head.crc == get_CRC8_check_sum((uint8_t*)&dartRemainingTimeMsg, (sizeof(dartRemainingTimeMsg.head) - sizeof(dartRemainingTimeMsg.head.crc)), 0xff)) && (dartRemainingTimeMsg.crc == get_CRC16_check_sum((uint8_t*)&dartRemainingTimeMsg, (sizeof(dartRemainingTimeMsg) - sizeof(dartRemainingTimeMsg.crc)), 0xffff)))
            {
                //cout << gameStatusMsgs.data.game_progress << endl;
                //这里还没写完，需要新的消息。
            }
            if((robotHealthMsgs.head.crc == get_CRC8_check_sum((uint8_t*)&robotHealthMsgs, (sizeof(robotHealthMsgs.head) - sizeof(robotHealthMsgs.head.crc)), 0xff)) && (robotHealthMsgs.crc == get_CRC16_check_sum((uint8_t*)&robotHealthMsgs, (sizeof(robotHealthMsgs) - sizeof(robotHealthMsgs.crc)), 0xffff)))
            {
                //cout << robotHealthMsgs.data.game_progress << endl;
                //这里还没写完，需要新的消息。
            }
            if((gameResultMsg.head.crc == get_CRC8_check_sum((uint8_t*)&gameResultMsg, (sizeof(gameResultMsg.head) - sizeof(gameResultMsg.head.crc)), 0xff)) && (gameResultMsg.crc == get_CRC16_check_sum((uint8_t*)&gameResultMsg, (sizeof(gameResultMsg) - sizeof(gameResultMsg.crc)), 0xffff)))
            {
                //cout << gameResultMsg.data.game_progress << endl;
                //这里还没写完，需要新的消息。
            }
            if((siteEventMsgs.head.crc == get_CRC8_check_sum((uint8_t*)&siteEventMsgs, (sizeof(siteEventMsgs.head) - sizeof(siteEventMsgs.head.crc)), 0xff)) && (siteEventMsgs.crc == get_CRC16_check_sum((uint8_t*)&siteEventMsgs, (sizeof(siteEventMsgs) - sizeof(siteEventMsgs.crc)), 0xffff)))
            {
                //cout << gameStatusMsgs.data.game_progress << endl;
                //这里还没写完，需要新的消息。
            }
            if((supplyProjectileActionMsg.head.crc == get_CRC8_check_sum((uint8_t*)&supplyProjectileActionMsg, (sizeof(supplyProjectileActionMsg.head) - sizeof(supplyProjectileActionMsg.head.crc)), 0xff)) && (supplyProjectileActionMsg.crc == get_CRC16_check_sum((uint8_t*)&supplyProjectileActionMsg, (sizeof(supplyProjectileActionMsg) - sizeof(supplyProjectileActionMsg.crc)), 0xffff)))
            {
                //cout << gameStatusMsgs.data.game_progress << endl;
                //这里还没写完，需要新的消息。
            }
            if((refereeWarningMsg.head.crc == get_CRC8_check_sum((uint8_t*)&refereeWarningMsg, (sizeof(refereeWarningMsg.head) - sizeof(refereeWarningMsg.head.crc)), 0xff)) && (refereeWarningMsg.crc == get_CRC16_check_sum((uint8_t*)&refereeWarningMsg, (sizeof(refereeWarningMsg) - sizeof(refereeWarningMsg.crc)), 0xffff)))
            {
                //cout << gameStatusMsgs.data.game_progress << endl;
                //这里还没写完，需要新的消息。
            }
            if((dartRemainingTimeMsg.head.crc == get_CRC8_check_sum((uint8_t*)&dartRemainingTimeMsg, (sizeof(dartRemainingTimeMsg.head) - sizeof(dartRemainingTimeMsg.head.crc)), 0xff)) && (dartRemainingTimeMsg.crc == get_CRC16_check_sum((uint8_t*)&dartRemainingTimeMsg, (sizeof(dartRemainingTimeMsg) - sizeof(dartRemainingTimeMsg.crc)), 0xffff)))
            {
                //cout << gameStatusMsgs.data.game_progress << endl;
                //这里还没写完，需要新的消息。
            }
            return true;
        }
        return false;
    }
};
serial_port sp;
vector<car_point>worldPoints;

void worldPointsCallback(const radar_msgs::points& msg)
{
    static int pub_count = 1;
    if(sp.is_enemy_red)
    {
        for(int i = 0; i < msg.data.size(); i++)
        {
            if(msg.color == "red")
            {
                car_point carPoint;
                carPoint.id = pub_count;
                pub_count++;
                carPoint.color = 0;
                carPoint.point = Point((msg.data[i].x * 15.0), (msg.data[i].y * 28.0));
                worldPoints.insert(worldPoints.begin(), carPoint);
            }
            else
            {
                pub_count = 1;
            }
        }
    }
    else
    {
        for(int i = 0; i < msg.data.size(); i++)
        {
            if(msg.color == "red")
            {
                pub_count = 1;
            }
            else if(msg.color == "blue")
            {
                car_point carPoint;
                carPoint.color = 1;
                carPoint.point = Point(msg.data[i].x * 15.0, msg.data[i].y * 28.0);
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
    string exchange;
    ros::param::get("battle_color", exchange);
    if(exchange == "red")
    {
        sp.is_enemy_red = false;
    }
    else
    {
        sp.is_enemy_red = true;
    }
    ros::Subscriber worldPointSub = nh.subscribe("/world_point", 1, &worldPointsCallback);
    ros::Rate loop(10);
    ROS_INFO_STREAM("Looping! ");
    while(ros::ok())
    {
        if(!worldPoints.empty())
        {
            if(worldPoints[0].color)
            {
                sp.sendMapMsgs(100 + worldPoints[0].id, worldPoints[0].point.x, worldPoints[0].point.y);
            }
            else
            {
                sp.sendMapMsgs(worldPoints[0].id, worldPoints[0].point.x, worldPoints[0].point.y);
            }
            worldPoints.erase(worldPoints.begin());
        }
        else
        {
            ros::spinOnce();
            /*for(int i = 0; i < 10; i++)
            {
                car_point carPoint;
                carPoint.point = Point2f(1.4 * i, 2.8 * i);
                if(i < 5)
                {
                    carPoint.color = true;
                }
                else
                {
                    carPoint.color = false;
                }
                worldPoints.push_back(carPoint);
            }*/
            //测试用
        }
        uint8_t test[113] = {1};
        sp.receiveMsgs();
        sp.sendInteractiveMsgs(test, 7);
        sp.sendInteractiveMsgs(test, 1);
        //循环休眠
        loop.sleep();
    }
    return 0;
}