#include <vector>

using std::vector;

typedef unsigned char uint8;
typedef unsigned short uint16;

enum class RobotType {
    hero = 1,
    engineer,
    infantry1,
    infantry2,
    infantry3,
    sentry = 7,
    outpost,
    base
};

uint8 getUid(uint8 team, uint8 type);

class Robot {
public:
    uint8 team;
    RobotType type;
    uint16 hpLimit;
    uint16 hpNow;

    Robot(uint8 team, RobotType type);
    void update(uint16 hp);
    uint8 getUid();
private:
    vector<uint16> hpLimits;
};
