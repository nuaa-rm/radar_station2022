//
// Created by dovejh on 2022/5/4.
//
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include "CRC8_CRC16.h"
#include "CRC.h"
#include "radar_msgs/points.h"
#include <opencv2/opencv.hpp>
#include "radar_msgs/game_state.h"
#include "radar_msgs/supply_projectile_action.h"
#include "radar_msgs/referee_warning.h"

using namespace std;
using namespace cv;

//消息头
struct frame_header
{
    uint8_t SOF = 0xA5;//固定值
    uint16_t data_length = 10;//data的长度
    uint8_t seq; //包序号
    uint8_t crc; //帧头crc8
} __attribute__((packed));

//小地图消息数据 10hz 发送
struct map_data
{
    uint16_t target_robot_id;
    float target_position_x;
    float target_position_y;
} __attribute__((packed));
struct map_msg {
    frame_header head;
    uint16_t cmd_id = 0x0305;
    map_data data;
    uint16_t crc;
} __attribute__((packed));

//车间通讯
struct robot_interactive_data//最大10HZ 发送和接收
{
    uint16_t cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t content[13];
} __attribute__((packed));

struct graphic_data_struct_t
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye:3;
    uint32_t graphic_tpye:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;
    uint32_t end_angle:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t radius:10;
    uint32_t end_x:11;
    uint32_t end_y:11;
} __attribute__((packed));
struct client_ui_data//最大10HZ 发送和接收
{
    uint16_t cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    graphic_data_struct_t data;
} __attribute__((packed));
struct client_ui_msgs {
    frame_header head;
    uint16_t cmd_id = 0x0301;
    client_ui_data data;
    uint16_t crc;
} __attribute__((packed));

struct _graphic_delete_t
{
    uint8_t operate_tpye;
    uint8_t layer;
} __attribute__((packed));
struct client_ui_delete_data//最大10HZ 发送和接收
{
    uint16_t cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    _graphic_delete_t data;
} __attribute__((packed));
struct client_ui_delete_msgs {
    frame_header head;
    uint16_t cmd_id = 0x0301;
    client_ui_delete_data data;
    uint16_t crc;
} __attribute__((packed));
struct robot_interactive_msgs {
    frame_header head;
    uint16_t cmd_id = 0x0301;
    robot_interactive_data data;
    uint16_t crc;
} __attribute__((packed));

//自定义控制消息
struct robot_interactive_control_data //30HZ 发送和接收
{
    uint8_t content[30];
} __attribute__((packed));
struct robot_interactive_control_msgs {
    frame_header head;
    uint16_t cmd_id = 0x0302;
    robot_interactive_control_data data;
    uint16_t crc;
} __attribute__((packed));

//车辆血量消息
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

//比赛状态信息
struct game_status_data //1hz 接收
{
    uint8_t game_type: 4; //1：机甲大师赛 2：单项赛 3：人工智能挑战赛 4：联盟赛3v3 5：联盟赛1v1
    uint8_t game_progress: 4; //0：未开始比赛 1：准备阶段 2：自检阶段 3：5s倒计时 4：对战中 5：比赛结算中
    uint16_t stage_remain_time = 0; //当前阶段剩余时间，单位s
    uint64_t SyncTimeStamp = 0; //机器人接收到该指令的精确Unix时间,当机载端收到有效的NTP服务器授时后生效

} __attribute__((packed));
struct game_status_msgs {
    frame_header head;
    uint16_t cmd_id = 0x0001;
    game_status_data data;
    uint16_t crc;
} __attribute__((packed));

//比赛结果
struct game_result_data//比赛结束发送 接收
{
    uint8_t winner; //0平局 1红方胜利 2蓝方胜利
} __attribute__((packed));
struct game_result_msg {
    frame_header head;
    uint16_t cmd_id = 0x0002;
    game_result_data data;
    uint16_t crc;
} __attribute__((packed));

//飞镖交互消息
struct dart_interactivate_msg_data {
    uint8_t dart_launch_opening_status; //fei biao fa she kou zhuang tai 1 guan bi 2 zheng zai huo dong 0 yi jing kai qi
    uint8_t dart_attack_target; //飞镖的打击目标,默认为前哨站;0:前哨站; 1:基地。
    uint16_t target_change_time; //切换打击目标时的比赛剩余时间,单位秒,从未切换默认为 0。
    uint16_t operate_launch_cmd_time; //最近一次操作手确定发射指令时的比赛剩余时间,单位秒, 初始值为 0。
} __attribute__((packed));

//场地机关占领消息
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
struct site_event_msgs {
    frame_header head;
    uint16_t cmd_id = 0x0101;
    site_event_data data;
    uint16_t crc;
} __attribute__((packed));

//补给站消息
struct supply_projectile_action_data //触发时发送 接收
{
    uint8_t supply_projectile_id; //补给站口ID 1一号补给口 2二号补给口
    uint8_t supply_robot_id; //补弹机器人ID
    uint8_t supply_projectile_step; //出弹口开闭状态 0关闭 1子弹准备中 2子弹下落
    uint8_t supply_projectile_num; //补单数量 50:50颗子弹 100:100颗子弹 150...... 200......
} __attribute__((packed));
struct supply_projectile_action_msg {
    frame_header head;
    uint16_t cmd_id = 0x0102;
    supply_projectile_action_data data;
    uint16_t crc;
} __attribute__((packed));

//裁判判罚消息
struct referee_warning_data //触发时发送 接收
{
    uint8_t level; //1黄牌 2红牌 3判负
    uint8_t foul_robot_id; //犯规机器人ID 判负时为0
} __attribute__((packed));
struct referee_warning_msg {
    frame_header head;
    uint16_t cmd_id = 0x0104;
    referee_warning_data data;
    uint16_t crc;
} __attribute__((packed));

//飞镖闸门关闭倒计时
struct dart_remaining_time_data //1hz 接收
{
    uint8_t dart_remaining_time; //15s倒计时
} __attribute__((packed));
struct dart_remaining_time_msg {
    frame_header head;
    uint16_t cmd_id = 0x0105;
    dart_remaining_time_data data;
    uint16_t crc;
} __attribute__((packed));
struct car_point {
    uint16_t id;
    Point2f point;
    bool color; //红色为0 蓝色为1
};
uint8_t warn_state;

/**
 * 串口通讯类
 */
class serial_port {
public:
    serial::Serial ser;
    map_msg mapMsg;
    robot_interactive_msgs robotInteractiveMsgs;
    robot_interactive_msgs HeroMsgs;
    robot_interactive_control_msgs robotInteractiveControlMsgs;
    robot_health_msgs robotHealthMsgs;
    game_result_msg gameResultMsg;
    site_event_msgs siteEventMsgs;
    supply_projectile_action_msg supplyProjectileActionMsg;
    referee_warning_msg refereeWarningMsg;
    dart_remaining_time_msg dartRemainingTimeMsg;
    game_status_msgs gameStatusMsgs;
    ros::Publisher gameStatePub;
    ros::Publisher supplyProjectileActionPub;
    ros::Publisher refereeWarningPub;
    radar_msgs::game_state gameStateRosMsg;
    radar_msgs::supply_projectile_action supplyProjectileActionRosMsg;
    radar_msgs::referee_warning refereeWarningRosMsg;
    uint8_t receiveData[1024];
    bool is_enemy_red = false;

    /**
     * 串口初始化
     * @return
     */
    int serial_port_init() {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }

    serial_port() {
        serial_port_init();
    }

    /**
     * 发送小地图消息
     * @param id 车辆ID
     * @param x x坐标
     * @param y y坐标
     * @return 是否成功发送
     */
    bool sendMapMsgs(uint16_t id, float x, float y) {
        mapMsg.head.SOF = 0xA5;
        mapMsg.head.data_length = 10;
        mapMsg.head.seq = 1;
        mapMsg.head.crc = get_CRC8_check_sum((uint8_t *) &mapMsg, (sizeof(mapMsg.head) - sizeof(mapMsg.head.crc)),
                                             0xff);
        mapMsg.cmd_id = 0x0305;
        mapMsg.data.target_position_x = x;
        mapMsg.data.target_position_y = y;
        mapMsg.data.target_robot_id = id;
        mapMsg.crc = get_CRC16_check_sum((uint8_t *) &mapMsg, (sizeof(mapMsg) - sizeof(mapMsg.crc)), 0xffff);
        ser.write((uint8_t *) &mapMsg, sizeof(map_msg));
        cout << "Send one map msg target_id = " << mapMsg.data.target_robot_id << " x = "
             << mapMsg.data.target_position_x << " y = " << mapMsg.data.target_position_y << endl;
        return true;
    }

    /**
     * 发送车间交互消息
     * @param receiver_id 在当接受者为红方时的ID
     * @return 是否成功发送
     */
    bool sendInteractiveMsgs(uint16_t receiver_id)//接受者ID以红方为准
    {
        //构造头
        robotInteractiveMsgs.head.SOF = 0xA5;
        robotInteractiveMsgs.head.data_length = sizeof(robot_interactive_data);
        robotInteractiveMsgs.head.seq = 1;
        robotInteractiveMsgs.head.crc = get_CRC8_check_sum((uint8_t *) &robotInteractiveMsgs,
                                                           (sizeof(robotInteractiveMsgs.head) -
                                                            sizeof(robotInteractiveMsgs.head.crc)), 0xff);
        robotInteractiveMsgs.cmd_id = 0x0301;
        robotInteractiveMsgs.data.cmd_id = 0x0201;
        //根据阵营自动调整ID
        if (is_enemy_red) {
            robotInteractiveMsgs.data.sender_id = 109;
            robotInteractiveMsgs.data.receiver_id = 100 + receiver_id;
        } else {
            robotInteractiveMsgs.data.sender_id = 9;
            robotInteractiveMsgs.data.receiver_id = receiver_id;
        }
        cout << robotInteractiveMsgs.data.sender_id << robotInteractiveMsgs.data.receiver_id << endl;
        robotInteractiveMsgs.crc = get_CRC16_check_sum((uint8_t *) &robotInteractiveMsgs,
                                                       (sizeof(robotInteractiveMsgs) -
                                                        sizeof(robotInteractiveMsgs.crc)), 0xffff);
        ser.write((uint8_t *) &robotInteractiveMsgs, sizeof(robotInteractiveMsgs));
        cout << "Send one interactive msg " << endl;
        std::cout << robotInteractiveMsgs.data.receiver_id << std::endl;
        return true;
    }
    /**
     * 发送与英雄交互的消息
     * @return 是否成功发送
     */
    bool sendHeroMsgs(){
        //构造头
        HeroMsgs.head.SOF = 0xA5;
        HeroMsgs.head.data_length = sizeof(robot_interactive_data);
        HeroMsgs.head.seq = 1;
        HeroMsgs.head.crc = get_CRC8_check_sum((uint8_t *) &HeroMsgs,
                                                           (sizeof(HeroMsgs.head) -
                                                            sizeof(HeroMsgs.head.crc)), 0xff);
        HeroMsgs.cmd_id = 0x0301;
        HeroMsgs.data.cmd_id = 0x0202;
        if (is_enemy_red) {
            HeroMsgs.data.sender_id = 109;
            HeroMsgs.data.receiver_id = 100 + 1;
        } else {
            HeroMsgs.data.sender_id = 9;
            HeroMsgs.data.receiver_id = 1;
        }
        cout << HeroMsgs.data.sender_id << HeroMsgs.data.receiver_id << endl;
        HeroMsgs.crc = get_CRC16_check_sum((uint8_t *) &HeroMsgs,
                                                       (sizeof(HeroMsgs) -
                                                        sizeof(HeroMsgs.crc)), 0xffff);
        ser.write((uint8_t *) &HeroMsgs, sizeof(HeroMsgs));
        cout << "Send one Hero msg " << endl;
        std::cout << HeroMsgs.data.receiver_id << std::endl;
        return true;
    }

    /**
     * 接收消息
     * @return 是否成功接收
     */
    bool receiveMsgs() {
        if (ser.available()) {
            bool if_pub = false;
            ser.read(receiveData, ser.available());
            //使用所有消息类型进行匹配，若CRC校验通过，则匹配成功
            gameStatusMsgs = (*(game_status_msgs *) receiveData);
            dartRemainingTimeMsg = (*(dart_remaining_time_msg *) receiveData);
            robotHealthMsgs = (*(robot_health_msgs *) receiveData);
            gameResultMsg = (*(game_result_msg *) receiveData);
            siteEventMsgs = (*(site_event_msgs *) receiveData);
            supplyProjectileActionMsg = (*(supply_projectile_action_msg *) receiveData);
            refereeWarningMsg = (*(referee_warning_msg *) receiveData);
            dartRemainingTimeMsg = (*(dart_remaining_time_msg *) receiveData);

            gameStateRosMsg.dart_remaining_time = 16;
            gameStateRosMsg.winner = 3;
            if ((gameStatusMsgs.head.crc == get_CRC8_check_sum((uint8_t *) &gameStatusMsgs,
                                                               (sizeof(gameStatusMsgs.head) -
                                                                sizeof(gameStatusMsgs.head.crc)), 0xff)) &&
                (gameStatusMsgs.crc ==
                 get_CRC16_check_sum((uint8_t *) &gameStatusMsgs, (sizeof(gameStatusMsgs) - sizeof(gameStatusMsgs.crc)),
                                     0xffff))) {

                gameStateRosMsg.game_progress = gameStatusMsgs.data.game_progress;
                gameStateRosMsg.game_type = gameStatusMsgs.data.game_type;
                gameStateRosMsg.stage_remain_time = gameStatusMsgs.data.stage_remain_time;
                if_pub = true;
            }
            if ((dartRemainingTimeMsg.head.crc == get_CRC8_check_sum((uint8_t *) &dartRemainingTimeMsg,
                                                                     (sizeof(dartRemainingTimeMsg.head) -
                                                                      sizeof(dartRemainingTimeMsg.head.crc)), 0xff)) &&
                (dartRemainingTimeMsg.crc == get_CRC16_check_sum((uint8_t *) &dartRemainingTimeMsg,
                                                                 (sizeof(dartRemainingTimeMsg) -
                                                                  sizeof(dartRemainingTimeMsg.crc)), 0xffff))) {
                gameStateRosMsg.dart_remaining_time = dartRemainingTimeMsg.data.dart_remaining_time;
                if_pub = true;
            }
            if ((robotHealthMsgs.head.crc == get_CRC8_check_sum((uint8_t *) &robotHealthMsgs,
                                                                (sizeof(robotHealthMsgs.head) -
                                                                 sizeof(robotHealthMsgs.head.crc)), 0xff)) &&
                (robotHealthMsgs.crc == get_CRC16_check_sum((uint8_t *) &robotHealthMsgs,
                                                            (sizeof(robotHealthMsgs) - sizeof(robotHealthMsgs.crc)),
                                                            0xffff))) {
                gameStateRosMsg.blue_1_robot_HP = robotHealthMsgs.data.blue_1_robot_HP;
                gameStateRosMsg.blue_2_robot_HP = robotHealthMsgs.data.blue_2_robot_HP;
                gameStateRosMsg.blue_3_robot_HP = robotHealthMsgs.data.blue_3_robot_HP;
                gameStateRosMsg.blue_4_robot_HP = robotHealthMsgs.data.blue_4_robot_HP;
                gameStateRosMsg.blue_5_robot_HP = robotHealthMsgs.data.blue_5_robot_HP;
                gameStateRosMsg.blue_7_robot_HP = robotHealthMsgs.data.blue_7_robot_HP;
                gameStateRosMsg.blue_base_HP = robotHealthMsgs.data.blue_base_HP;
                gameStateRosMsg.blue_outpose_HP = robotHealthMsgs.data.blue_outpose_HP;
                gameStateRosMsg.red_1_robot_HP = robotHealthMsgs.data.red_1_robot_HP;
                gameStateRosMsg.red_2_robot_HP = robotHealthMsgs.data.red_2_robot_HP;
                gameStateRosMsg.red_3_robot_HP = robotHealthMsgs.data.red_3_robot_HP;
                gameStateRosMsg.red_4_robot_HP = robotHealthMsgs.data.red_4_robot_HP;
                gameStateRosMsg.red_5_robot_HP = robotHealthMsgs.data.red_5_robot_HP;
                gameStateRosMsg.red_7_robot_HP = robotHealthMsgs.data.red_7_robot_HP;
                gameStateRosMsg.red_base_HP = robotHealthMsgs.data.red_base_HP;
                gameStateRosMsg.red_outpose_HP = robotHealthMsgs.data.red_outpose_HP;
                if_pub = true;
            }
            if ((gameResultMsg.head.crc == get_CRC8_check_sum((uint8_t *) &gameResultMsg, (sizeof(gameResultMsg.head) -
                                                                                           sizeof(gameResultMsg.head.crc)),
                                                              0xff)) && (gameResultMsg.crc ==
                                                                         get_CRC16_check_sum((uint8_t *) &gameResultMsg,
                                                                                             (sizeof(gameResultMsg) -
                                                                                              sizeof(gameResultMsg.crc)),
                                                                                             0xffff))) {
                gameStateRosMsg.winner = gameResultMsg.data.winner;
                if_pub = true;
            }
            if ((siteEventMsgs.head.crc == get_CRC8_check_sum((uint8_t *) &siteEventMsgs, (sizeof(siteEventMsgs.head) -
                                                                                           sizeof(siteEventMsgs.head.crc)),
                                                              0xff)) && (siteEventMsgs.crc ==
                                                                         get_CRC16_check_sum((uint8_t *) &siteEventMsgs,
                                                                                             (sizeof(siteEventMsgs) -
                                                                                              sizeof(siteEventMsgs.crc)),
                                                                                             0xffff))) {
                gameStateRosMsg.if_supply_projectile_one_occupied = (siteEventMsgs.data.event_type | 0x80000000);
                gameStateRosMsg.if_supply_projectile_two_occupied = (siteEventMsgs.data.event_type | 0x40000000);
                gameStateRosMsg.if_supply_projectile_three_occupied = (siteEventMsgs.data.event_type | 0x20000000);
                gameStateRosMsg.if_wind_mill_hit_place_occupied = (siteEventMsgs.data.event_type | 0x10000000);
                gameStateRosMsg.if_wind_mill_big_lighted = (siteEventMsgs.data.event_type | 0x80000000);
                gameStateRosMsg.if_wind_mill_small_lighted = (siteEventMsgs.data.event_type | 0x08000000);
                gameStateRosMsg.if_RB2_occupied = (siteEventMsgs.data.event_type | 0x04000000);
                gameStateRosMsg.if_RB3_occupied = (siteEventMsgs.data.event_type | 0x02000000);
                gameStateRosMsg.if_RB4_occupied = (siteEventMsgs.data.event_type | 0x01000000);
                gameStateRosMsg.if_base_protected = (siteEventMsgs.data.event_type | 0x00800000);
                gameStateRosMsg.if_outpose_alive = (siteEventMsgs.data.event_type | 0x00400000);
                if_pub = true;
            }
            if ((supplyProjectileActionMsg.head.crc == get_CRC8_check_sum((uint8_t *) &supplyProjectileActionMsg,
                                                                          (sizeof(supplyProjectileActionMsg.head) -
                                                                           sizeof(supplyProjectileActionMsg.head.crc)),
                                                                          0xff)) && (supplyProjectileActionMsg.crc ==
                                                                                     get_CRC16_check_sum(
                                                                                             (uint8_t *) &supplyProjectileActionMsg,
                                                                                             (sizeof(supplyProjectileActionMsg) -
                                                                                              sizeof(supplyProjectileActionMsg.crc)),
                                                                                             0xffff))) {
                supplyProjectileActionRosMsg.supply_projectile_id = supplyProjectileActionMsg.data.supply_projectile_id;
                supplyProjectileActionRosMsg.supply_robot_id = supplyProjectileActionMsg.data.supply_robot_id;
                supplyProjectileActionRosMsg.supply_projectile_step = supplyProjectileActionMsg.data.supply_projectile_step;
                supplyProjectileActionRosMsg.supply_projectile_num = supplyProjectileActionMsg.data.supply_projectile_num;
                if_pub = true;
            }
            if ((refereeWarningMsg.head.crc == get_CRC8_check_sum((uint8_t *) &refereeWarningMsg,
                                                                  (sizeof(refereeWarningMsg.head) -
                                                                   sizeof(refereeWarningMsg.head.crc)), 0xff)) &&
                (refereeWarningMsg.crc == get_CRC16_check_sum((uint8_t *) &refereeWarningMsg,
                                                              (sizeof(refereeWarningMsg) -
                                                               sizeof(refereeWarningMsg.crc)), 0xffff))) {
                refereeWarningRosMsg.level = refereeWarningMsg.data.level;
                refereeWarningRosMsg.foul_robot_id = refereeWarningMsg.data.foul_robot_id;
                if_pub = true;
            }
            if ((dartRemainingTimeMsg.head.crc == get_CRC8_check_sum((uint8_t *) &dartRemainingTimeMsg,
                                                                     (sizeof(dartRemainingTimeMsg.head) -
                                                                      sizeof(dartRemainingTimeMsg.head.crc)), 0xff)) &&
                (dartRemainingTimeMsg.crc == get_CRC16_check_sum((uint8_t *) &dartRemainingTimeMsg,
                                                                 (sizeof(dartRemainingTimeMsg) -
                                                                  sizeof(dartRemainingTimeMsg.crc)), 0xffff))) {
                gameStateRosMsg.dart_remaining_time = dartRemainingTimeMsg.data.dart_remaining_time;
                if_pub = true;
            }
            if (if_pub) {
                gameStatePub.publish(gameStateRosMsg);
                return true;
            }
            return false;
        }
        return false;
    }
};

serial_port sp;
vector<car_point> worldPoints;

/**
 * 目标世界坐标回调函数
 * @param msg 收到的消息
 */
void worldPointsCallback(const radar_msgs::points &msg) {
    warn_state = msg.id;
    static int pubCount = 0;
    if (sp.is_enemy_red) {
        for (int i = 0; i < msg.data.size(); i++) {
            if (msg.data[i].id <= 4) {
                car_point carPoint;
                carPoint.id = msg.data[i].id + 1;
                carPoint.color = 0;
                carPoint.point = Point2f((msg.data[i].x * 15.0), (msg.data[i].y * 28.0));
                worldPoints.insert(worldPoints.begin(), carPoint);
            } else if (msg.data[i].id == 5) {
                car_point carPoint;
                carPoint.id = 7;
                carPoint.color = 0;
                carPoint.point = Point((msg.data[i].x * 15.0), (msg.data[i].y * 28.0));
                worldPoints.insert(worldPoints.begin(), carPoint);
            } else if (msg.data[i].id == 12) {
                car_point carPoint;
                if (pubCount == 0) {
                    carPoint.id = 6;
                } else if (pubCount == 1) {
                    carPoint.id = 9;
                } else if (pubCount == 2) {
                    carPoint.id = 10;
                } else if (pubCount == 3) {
                    carPoint.id = 11;
                }
                if (pubCount >= 3) {
                    pubCount = 0;
                } else {
                    pubCount++;
                }
                carPoint.color = 0;
                carPoint.point = Point((msg.data[i].x * 15.0), (msg.data[i].y * 28.0));
                worldPoints.insert(worldPoints.begin(), carPoint);
            }
        }
    } else {
        for (int i = 0; i < msg.data.size(); i++) {
            if (msg.data[i].id >= 6 || msg.data[i].id <= 10) {
                car_point carPoint;
                carPoint.id = msg.data[i].id - 5;
                carPoint.color = 1;
                carPoint.point = Point((msg.data[i].x * 15.0), (msg.data[i].y * 28.0));
                worldPoints.insert(worldPoints.begin(), carPoint);
            } else if (msg.data[i].id == 11) {
                car_point carPoint;
                carPoint.id = 7;
                carPoint.color = 1;
                carPoint.point = Point((msg.data[i].x * 15.0), (msg.data[i].y * 28.0));
                worldPoints.insert(worldPoints.begin(), carPoint);
            } else if (msg.data[i].id == 13) {
                car_point carPoint;
                carPoint.id = 6;
                if (pubCount == 0) {
                    carPoint.id = 6;
                } else if (pubCount == 1) {
                    carPoint.id = 9;
                } else if (pubCount == 2) {
                    carPoint.id = 10;
                } else if (pubCount == 3) {
                    carPoint.id = 11;
                }
                if (pubCount >= 3) {
                    pubCount = 0;
                } else {
                    pubCount++;
                }
                carPoint.color = 1;
                carPoint.point = Point((msg.data[i].x * 15.0), (msg.data[i].y * 28.0));
                worldPoints.insert(worldPoints.begin(), carPoint);
            }
        }
    }
}
/**
 * 哨兵相关预警消息的接受
 * @param msg
 */
void GuardCallback(const radar_msgs::points &msg) {
    sp.robotInteractiveMsgs.data.content[0]=0xcc;
    sp.robotInteractiveMsgs.data.content[1]=(int16_t)msg.data[0].x;
    sp.robotInteractiveMsgs.data.content[2]=(int16_t)msg.data[0].x>>8;
    sp.robotInteractiveMsgs.data.content[3]=(int16_t)msg.data[0].y;
    sp.robotInteractiveMsgs.data.content[4]=(int16_t)msg.data[0].y>>8;
    sp.robotInteractiveMsgs.data.content[5]=(int16_t)msg.data[0].z;
    sp.robotInteractiveMsgs.data.content[6]=(int16_t)msg.data[0].z>>8;
    sp.robotInteractiveMsgs.data.content[7]=(int16_t)msg.data[1].x;
    sp.robotInteractiveMsgs.data.content[8]=(int16_t)msg.data[1].x>>8;
    sp.robotInteractiveMsgs.data.content[9]=(int16_t)msg.data[1].y;
    sp.robotInteractiveMsgs.data.content[10]=(int16_t)msg.data[1].y>>8;
    sp.robotInteractiveMsgs.data.content[11]=(int16_t)msg.data[1].z;
    sp.robotInteractiveMsgs.data.content[12]=(int16_t)msg.data[1].z>>8;
}
//英雄相关预警消息的接收
void HeroCallback(const radar_msgs::point &msg) {
    sp.HeroMsgs.data.content[0]=0xbb;
    sp.HeroMsgs.data.content[1]=(int16_t)msg.x;
    sp.HeroMsgs.data.content[2]=((int16_t)msg.x)>>8;
    sp.HeroMsgs.data.content[3]=(uint8_t)(100*(msg.x-(int16_t)msg.x));
}
int main(int argc, char **argv) {
    //初始化节点
    ros::init(argc, argv, "serial_port_node");
    //声明节点句柄
    ros::NodeHandle nh;

    sp.gameStatePub = nh.advertise<radar_msgs::game_state>("game_state", 1);
    sp.supplyProjectileActionPub = nh.advertise<radar_msgs::supply_projectile_action>("supply_projectile_action", 1);
    sp.refereeWarningPub = nh.advertise<radar_msgs::referee_warning>("referee_warning", 1);
    if (!sp.ser.isOpen()) {
        ROS_ERROR_STREAM("Unable to open port, please check USB2TTL! ");
        return -1;
    } else {
        ROS_INFO_STREAM("Serial Port initialized! ");
    }
    //读取我方阵营参数
    string exchange;
    ros::param::get("battle_state/battle_color", exchange);
    if (exchange == "red") {
        sp.is_enemy_red = false;
    } else {
        sp.is_enemy_red = true;
    }
    ros::Subscriber worldPointSub = nh.subscribe("/world_point", 1, &worldPointsCallback);
    ros::Subscriber GuardSub = nh.subscribe("/guard_pub", 1, &GuardCallback);
    ros::Subscriber HeroSub = nh.subscribe("/hero_pub", 1, &HeroCallback);
    以100hz定频循环
    ros::Rate loop(100);
    ROS_INFO_STREAM("Looping! ");
    int count = 0;
    while (ros::ok()) {
        count++;
        //分频为10hz，实现10hz定频发送
        if (count >= 10) {
            sp.sendInteractiveMsgs(7);
            sp.sendHeroMsgs();
            //逐一发送小地图目标点，当等待发布的点为空时接收一次新的消息
            if (!worldPoints.empty()) {
                if (worldPoints[0].color) {
                    sp.sendMapMsgs(100 + worldPoints[0].id, worldPoints[0].point.x, worldPoints[0].point.y);
                } else {
                    sp.sendMapMsgs(worldPoints[0].id, worldPoints[0].point.x, worldPoints[0].point.y);
                }
                worldPoints.erase(worldPoints.begin());
            } else {
                ros::spinOnce();
//                for (int i = 0; i < 10; i++) {
//                    car_point carPoint;
//                    carPoint.point = Point2f(1.4 * i, 2.8 * i);
//                    carPoint.id = 6;
//                    if (i < 5) {
//                        carPoint.color = true;
//                    } else {
//                        carPoint.color = false;
//                    }
//                    worldPoints.push_back(carPoint);
//                }
                //测试用
            }
            count = 0;
        }
        sp.receiveMsgs();
        ros::spinOnce();
        //循环休眠
        loop.sleep();
    }
    return 0;
}
