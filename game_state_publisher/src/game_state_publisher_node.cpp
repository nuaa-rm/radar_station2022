#include <ros/ros.h>
#include <radar_msgs/game_state.h>
#include <radar_msgs/referee_warning.h>
#include <radar_msgs/supply_projectile_action.h>
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "game_state_publisher_node");
    ros::NodeHandle n;
    ros::Publisher game_state_pub = n.advertise<radar_msgs::game_state>("/game_state", 1);
    ros::Rate loop_rate(1);
    radar_msgs::game_state msg_game_state;
    int i = 10;
    while(ros::ok)
    {
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
         if(i < 0)
         {
             i = 10;
         }
         game_state_pub.publish(msg_game_state);
         loop_rate.sleep();
    }
}
