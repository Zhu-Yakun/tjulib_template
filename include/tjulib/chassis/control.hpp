#pragma once
#include "vex.h"
#include "straightChassis.hpp"
#include <cmath>
#include <vector>

namespace tjulib {
    const double PI = 3.14159265358979323846;
    //控制转向响应的非线性参数
    //较小的值将导致转向响应更加线性，较大的值将引入非线性，低速度下更容易微调
    const double CD_TURN_NONLINEARITY = 0.1;
    //操纵杆输入的死区
    const double DRIVE_DEADBAND = 0.5;
    //用于平滑控制底盘速度变化的参数。
    //控制了机器人加速和减速的速率。较大的值将导致速度变化更加迅速，而较小的值将使速度变化更加平滑
    const double DRIVE_SLEW = 0.1;
    //用于控制底盘在快速转向时的行为的参数。
    //调整了机器人对快速转向输入的响应。较大的值将导致机器人更积极地应对快速转向，但可能会引入过度振荡或不稳定性。较小的值会减缓机器人的响应。
    const double CD_NEG_INERTIA_SCALAR = 3.0;
    //控制机器人的响应灵敏度。较大的值将导致机器人对输入更为敏感，速度变化更迅速，而较小的值会导致响应变得更加迟缓
    const double CD_SENSITIVITY = 0.7;

    class Control : public straightChassis {
    public:
        Control(std::vector<vex::motor*>& leftMotors, std::vector<vex::motor*>& rightMotors)
            : straightChassis(leftMotors, rightMotors) {}

        double _turnRemapping(double iturn);
        void _updateAccumulators();
        void cheesyDrive(double ithrottle, double iturn);

    private:
        double quickStopAccumulator = 0.0;
        double negInertiaAccumulator = 0.0;
        double prevTurn = 0.0;
        double prevThrottle = 0.0;
    };
} // namespace tjulib