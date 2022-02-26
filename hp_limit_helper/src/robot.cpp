#include "hp_limit_helper/robot.h"

inline uint8 getUid(uint8 team, uint8 type) {
    return team * 10 + type;
}

Robot::Robot(uint8 team, RobotType type) {
    this->team = team;
    this->type = type;
    if (type == RobotType::hero) {
        hpLimits = vector<uint16>{150, 200, 250, 300, 350, 450};
    } else if (type == RobotType::engineer) {
        hpLimits = vector<uint16>{500};
    }  else if (type == RobotType::infantry1 || type == RobotType::infantry2 || type == RobotType::infantry3) {
        hpLimits = vector<uint16>{100, 150, 200, 250, 300, 400, 500};
    } else if (type == RobotType::sentry) {
        hpLimits = vector<uint16>{600};
    } else if (type == RobotType::outpost) {
        hpLimits = vector<uint16>{1500};
    } else if (type == RobotType::base) {
        hpLimits = vector<uint16>{5000};
    }
    hpLimit = hpLimits[0];
}

void Robot::update(uint16 hp) {
    if (hp > hpLimit) {
        for (const auto& limit : hpLimits) {
            if (hp <= limit) {
                hpLimit = limit;
                break;
            }
        }
    }
    hpNow = hp;
}

uint8 Robot::getUid() {
    return ::getUid(team, (uint8)type);
}
