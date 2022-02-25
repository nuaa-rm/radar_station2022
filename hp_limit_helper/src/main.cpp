#include <ros/ros.h>
#include <map>
#include "hp_limit_helper/robot.h"
#include "hp_limit_helper/RobotHP.h"
#include "hp_limit_helper/RobotsHP.h"

using std::map;
using std::pair;
using ros::Publisher;
using ros::Subscriber;
using ros::NodeHandle;
using ros::spinOnce;
using ros::Rate;
using hp_limit_helper::RobotsHP;
using hp_limit_helper::RobotHP;

typedef pair<uint8, Robot> RobotPair;

map<uint8, Robot> robots;
Publisher hpLimitsPublisher;


void robotHpCallback(const RobotsHP::ConstPtr &msg) {
    vector<RobotHP> data;
    for (const auto &it: msg->data) {
        uint8 uid = getUid(it.team, it.number);
        RobotHP robotHp;
        robotHp.team = it.team;
        robotHp.number = it.number;
        auto it_robot = robots.find(uid);
        if (it_robot != robots.end()) {
            it_robot->second.update(it.hp);
            robotHp.hp = it_robot->second.hpLimit;
        } else {
            Robot robot(it.team, (RobotType) it.number);
            robot.update(it.hp);
            robotHp.hp = robot.hpLimit;
            robots.insert(RobotPair(uid, robot));
        }
        data.push_back(robotHp);
    }
    RobotsHP msg_out;
    msg_out.data = data;
    hpLimitsPublisher.publish(msg_out);
}


int main() {
    NodeHandle nh;
    hpLimitsPublisher = nh.advertise<RobotsHP>("hp_limits", 1);
    Subscriber robotHpSubscriber = nh.subscribe("now_hp", 1, robotHpCallback);
    Rate rate(10);
    while (ros::ok()) {
        spinOnce();
        rate.sleep();
    }
    return 0;
}
