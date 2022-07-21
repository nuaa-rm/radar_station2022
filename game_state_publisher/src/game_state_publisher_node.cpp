#include <ros/ros.h>
#include <radar_msgs/game_state.h>
#include <radar_msgs/referee_warning.h>
#include <radar_msgs/supply_projectile_action.h>
#include <radar_msgs/point.h>
#include <radar_msgs/points.h>
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "game_state_publisher_node");
    ros::NodeHandle n;
    ros::Publisher game_state_pub = n.advertise<radar_msgs::game_state>("/game_state", 1);
    ros::Publisher world_point_pub = n.advertise<radar_msgs::points>("/world_point", 1);
    ros::Rate loop_rate(1);
    radar_msgs::game_state msg_game_state;
    radar_msgs::point one_point_msg;

    int i = 10;
    while(ros::ok)
    {
         radar_msgs::points world_points_msg;
         msg_game_state.game_type = 1;
         msg_game_state.red_base_HP = 5000;
         msg_game_state.blue_base_HP = 4500;
         msg_game_state.game_progress = 0;
         msg_game_state.red_1_robot_HP = 175;
         msg_game_state.red_2_robot_HP = 100;
         msg_game_state.red_3_robot_HP = 144;
         msg_game_state.red_4_robot_HP = 450;
         msg_game_state.red_5_robot_HP = 333;
         msg_game_state.red_7_robot_HP = 600;
         msg_game_state.red_outpose_HP = 1500;
         msg_game_state.blue_1_robot_HP = 80;
         msg_game_state.blue_2_robot_HP = 250;
         msg_game_state.blue_3_robot_HP = 250;
         msg_game_state.blue_4_robot_HP = 300;
         msg_game_state.blue_5_robot_HP = 288;
         msg_game_state.blue_7_robot_HP = 0;
         msg_game_state.blue_outpose_HP = 0;
         msg_game_state.stage_remain_time = i--;

         one_point_msg.x = 0.1 * i;
         one_point_msg.y = 0.12 * i;
         one_point_msg.id = 0;
         world_points_msg.data.push_back(one_point_msg);
         one_point_msg.x = 0.13 * i;
         one_point_msg.y = 0.14 * i;
         one_point_msg.id = 1;
         world_points_msg.data.push_back(one_point_msg);
         one_point_msg.x = 0.15 * i;
         one_point_msg.y = 0.01 * i;
         one_point_msg.id = 2;
         world_points_msg.data.push_back(one_point_msg);
         one_point_msg.x = 0.02 * i;
         one_point_msg.y = 0.03 * i;
         one_point_msg.id = 3;
         world_points_msg.data.push_back(one_point_msg);
         one_point_msg.x = 0.04 * i;
         one_point_msg.y = 0.05 * i;
         one_point_msg.id = 4;
         world_points_msg.data.push_back(one_point_msg);
         one_point_msg.x = 0.06 * i;
         one_point_msg.y = 0.07 * i;
         one_point_msg.id = 5;
         world_points_msg.data.push_back(one_point_msg);
         one_point_msg.x = 0.08 * i;
         one_point_msg.y = 0.09 * i;
         one_point_msg.id = 6;
         world_points_msg.data.push_back(one_point_msg);
         one_point_msg.x = 0.01 * i;
         one_point_msg.y = 0.1 * i;
         one_point_msg.id = 7;
         world_points_msg.data.push_back(one_point_msg);
         one_point_msg.x = 0.2 * i;
         one_point_msg.y = 0.09 * i;
         one_point_msg.id = 8;
         world_points_msg.data.push_back(one_point_msg);
         one_point_msg.x = 0.12 * i;
         one_point_msg.y = 0.01 * i;
         one_point_msg.id = 9;
         world_points_msg.data.push_back(one_point_msg);
         one_point_msg.x = 0.13 * i;
         one_point_msg.y = 0.07 * i;
         one_point_msg.id = 10;
         world_points_msg.data.push_back(one_point_msg);
         one_point_msg.x = 0.1 * i;
         one_point_msg.y = 0.06 * i;
         one_point_msg.id = 11;
         world_points_msg.data.push_back(one_point_msg);
         one_point_msg.x = 0.07 * i;
         one_point_msg.y = 0.03 * i;
         one_point_msg.id = 12;
         world_points_msg.data.push_back(one_point_msg);
         one_point_msg.x = 0.01 * i;
         one_point_msg.y = 0.09 * i;
         one_point_msg.id = 13;
         world_points_msg.data.push_back(one_point_msg);
         world_point_pub.publish(world_points_msg);
         if(i < 0)
         {
             i = 10;
         }
         game_state_pub.publish(msg_game_state);
         loop_rate.sleep();
    }
}
