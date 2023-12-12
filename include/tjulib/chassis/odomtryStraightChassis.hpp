#pragma once
#include "vex.h"
#include "straightChassis.hpp"
#include "tjulib/pidControl.hpp"
#include "../odom.hpp"
#include <vector>

typedef float T;

namespace tjulib
{
    class odomtryStraightChasiss : public straightChassis
    {
    private:
        // 与control.hpp中有重复定义，因此放到了private里面
        const double PI = 3.14159265358979323846;
        const T toDegrees = 180.0 / PI; // 弧度乘以该数字转为角度

        Odom *odom = NULL;
        pidControl *fwdControl = NULL;
        pidControl *turnControl = NULL;

        T angleWrap(T deg);
        T getDistanceTo(Point target);
        T getDegreeTo(Point target);

    public:
        odomtryStraightChasiss(std::vector<vex::motor*> & leftMotors, std::vector<vex::motor*> & rightMotors, Odom *odom, pidControl *fwdControl, pidControl *turnControl)
            : straightChassis(leftMotors, rightMotors), odom(odom), fwdControl(fwdControl), turnControl(turnControl){};
        void turnToTarget(Point target, T maxSpeed, double maxtime_ms, int fwd = 1);
        void moveToTarget(Point target, T maxFwdSpeed, T maxTurnSpeed, double maxtime_ms, int fwd = 1);
        void turnToAngle(double angle, T maxSpeed, double maxtime_ms, int fwd = 1);
    };
}