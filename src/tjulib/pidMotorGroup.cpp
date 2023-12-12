#include "tjulib/pidMotorGroup.hpp"
using namespace vex;
using namespace tjulib;

// 重置总距离为0
void resetTotalDeg(motor_group& motor_g)
{
    motor_g.resetPosition();
}

// 获取当前的总角度
double getTotalDistance(motor_group& motor_g)
{
    double distance = motor_g.position(deg);
    return distance;
}

// 将速度限制在指定的范围内
double keepInRange(double speed, double minSpeed, double maxSpeed)
{
    if (speed < minSpeed) {
        speed = minSpeed;
    } else if (speed > maxSpeed) {
        speed = maxSpeed;
    }

    return speed;
}

// 以最大速度为maxSpeed旋转目标角度（单位：度）
void pid_motor_group::rotateDegrees(double degrees, int maxSpeed)
{
    resetTotalDeg(*this);

    const double minSpeed = 3;// 电机的最低速度
    const double accelRate = 100;//加速率：改变加速/减速的速度
    const double deaccelRate = 4;//减速时的速度倍率（末尾的斜率）。较高时开始放慢速度

    double targetAngle = degrees;
    double error = targetAngle;
    double speed;

    while (fabs(error) > targetRange) {
        error = targetAngle - getTotalDistance(*this);

        if (fabs(error) > fabs(targetAngle/2)) {
            speed = (1 - (fabs(error) / fabs(targetAngle))) * accelRate;
        } else {
            speed = (fabs(error) / fabs(targetAngle)) * deaccelRate;
        }

        speed = fabs(speed);
        speed *= maxSpeed;
        speed = keepInRange(speed, minSpeed, maxSpeed);

        if (error < 0) {
            speed *= -1;
        }

        // 在这里执行旋转动作，具体实现取决于您的系统
        this->spin(forward,speed,pct);
            
        }

    // 停止旋转动作，具体实现取决于您的系统
    this->stop(brakeType::hold);

}


void pid_motor_group::set_params(double AminSpeed, double AaccelRate, double AdeaccelRate, double AtargetRange){
    minSpeed = AminSpeed;// 电机的最低速度
    accelRate = AaccelRate;//加速率：改变加速/减速的速度
    deaccelRate = AdeaccelRate;//减速时的速度倍率（末尾的斜率）。较高时开始放慢速度
    targetRange = AtargetRange;
}
