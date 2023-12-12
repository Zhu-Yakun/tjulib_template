#pragma once
#include "vex.h"


class pid_motor_group: public vex::motor_group{
    protected:
    double minSpeed = 3;// 电机的最低速度
    double accelRate = 100;//加速率：改变加速/减速的速度
    double deaccelRate = 20;//减速时的速度倍率（末尾的斜率）。较高时开始放慢速度
    double targetRange = 2;

    public:
    using motor_group::motor_group; 
    void rotateDegrees(double degrees, int maxSpeed);
    void set_params(double AminSpeed, double AaccelRate, double AdeaccelRate, double AtargetRange);
};

