#include"tjulib/pid.hpp"
#include"vex.h"
#include <cmath>
#include "tjulib/odom.hpp"
#include "utilities.h"
using namespace vex;
using namespace tjulib;

// 重置总距离为0
void resetTotalDeg(motor& motor_g)
{
    motor_g.resetPosition();
}

// 获取当前的总角度
double getTotalDistance(motor& motor_g)
{
    double distance = motor_g.position(deg);
    return distance;
}

// 将速度限制在指定的范围内
double keepInRange1(double speed, double minSpeed, double maxSpeed)
{
    if (speed < minSpeed) {
        speed = minSpeed;
    } else if (speed > maxSpeed) {
        speed = maxSpeed;
    }

    return speed;
}

PIDMotor::PIDMotor(int index, vex::gearSetting type, bool reverse, float Kp, float Ki, float Kd):motor(index,type, reverse), arg(this,&this->MPID,&targetRPM), spinPIDMotor(&processVelPID, &arg), MPID(Kp, Ki, Kd){
    spinPIDMotor.suspend();
}

void PIDMotor::setPID( float Kp, float Ki, float Kd){
    MPID.setGains(Kp, Ki, Kd);
}

//暂时只有pct的
void PIDMotor::spinPID( directionType dir, float velocity){
    //改参数
    targetRPM = velocity * (dir==directionType::fwd?1:-1);
    if(std::abs(velocity)<1e-6)
        spinPIDMotor.suspend();
    else
        spinPIDMotor.resume();
}

void PIDMotor::distancePID(){}

PIDMotor::~PIDMotor(){
    spinPIDMotor.stop();
}

//传地址，连带修改

// 以最大速度为maxSpeed旋转目标角度（单位：度）
void PIDMotor::rotateDegrees(double degrees, int maxSpeed)
{
    resetTotalDeg(*this);

    // const double minSpeed = 3;// 电机的最低速度
    // const double accelRate = 100;//加速率：改变加速/减速的速度
    // const double deaccelRate = 4;//减速时的速度倍率（末尾的斜率）。较高时开始放慢速度

    double targetAngle = degrees;
    double error = targetAngle;
    double speed;

    while (fabs(error) > targetRange) {
        error = targetAngle - getTotalDistance(*this);

        //打印
        printf("error: %lf\n", error);

        if (fabs(error) > fabs(targetAngle/2)) {
            speed = (1 - (fabs(error) / fabs(targetAngle))) * accelRate;
        } else {
            speed = (fabs(error) / fabs(targetAngle)) * deaccelRate;
        }

        printf("before correct speed: %lf\n", speed);


        speed = fabs(speed);
        speed *= maxSpeed;
        speed = keepInRange1(speed, minSpeed, maxSpeed);


        if (error < 0) {
            speed *= -1;
        }

        printf("speed: %lf\n", speed);


        // 在这里执行旋转动作，具体实现取决于您的系统
        this->spin(forward,speed,pct);
            
        }

    // 停止旋转动作，具体实现取决于您的系统
    this->stop();

}

void PIDMotor::set_distance_pid_params(double AminSpeed, double AaccelRate, double AdeaccelRate, double AtargetRange){
    minSpeed = AminSpeed;// 电机的最低速度
    accelRate = AaccelRate;//加速率：改变加速/减速的速度
    deaccelRate = AdeaccelRate;//减速时的速度倍率（末尾的斜率）。较高时开始放慢速度
    targetRange = AtargetRange;
}


float currentRPM, motorPower, lastPower;

int processVelPID(void * nargs){
    args *arg = (args *)nargs;
    while(true) {
        currentRPM = arg->pmotor->velocity(rpm);
        motorPower = arg->pMPID->calculate(*arg->ptargetRPM, currentRPM);
        
        if(motorPower <= 0) motorPower = 0; //Prevent motor from spinning backward
        
        //Give the motor a bit of a starting boost
        if(motorPower > lastPower && lastPower < 10 && motorPower > 10) lastPower = 10;
        
        lastPower = motorPower;
        
    
        vexMotorVoltageSet(arg->pmotor->index(),motorPower* Math::velocityToVoltage);//controled by voltage
        wait(10, msec);
    }
    return 0;
}




// 上赛季部分PID，已弃用
/*************************************************************************/

void VRUN(double L, double R){
  vexMotorVoltageSet(L1.index(),L* 12000.0 / 100.0);
  vexMotorVoltageSet(L2.index(),L* 12000.0 / 100.0);
  vexMotorVoltageSet(L3.index(),L* 12000.0 / 100.0);
  vexMotorVoltageSet(R1.index(),R* 12000.0 / 100.0);
  vexMotorVoltageSet(R2.index(),R* 12000.0 / 100.0);
  vexMotorVoltageSet(R3.index(),R* 12000.0 / 100.0);
}

void fieldCentricVRUN(double controllerX, double controllerY, Odom odom)
{
  double magnitude = sqrt(pow(controllerX, 2) + pow(controllerY, 2));
  double theta = atan2(controllerY, controllerX);
  double theta2 = theta + odom.globalPoint.angle;
  double x2 = magnitude * cos(theta2);
  double y2 = magnitude * sin(theta2);

  // Motor output
  double left = y2 + x2;// + turning;
  double right = y2 - x2;// - turning;

  VRUN(left, right);
}

PID::PID(double _kP, double _kD, double _turn_kP, double _turn_kD)
{
    // Movement PID constants
    kP = _kP;
    kD = _kD;
    // Movement PID
    derivative_x = 0.0;
    derivative_y = 0.0;
    prevError_x = 0.0;
    prevError_y = 0.0;
    // Turn PID constants
    turn_kP = _turn_kP;
    turn_kD = _turn_kD;
    // Turn PID
    turn_derivative = 0.0;
    turn_prevError = 0.0;
    // getError function - for debugging
    dist = 0.0;
    turn_e = 0.0;
    maxSpeed = 100;
}

void PID::small_Range_moveTo(double x, double y, double finalAngle, int maxTimeMs, Odom odom)
{
    double target_x = x;
    double target_y = y;              // Convert to inverted okapi y-direction
    double target_theta = finalAngle; // Convert to default okapi clockwise angle
    // PD parameters:
    //! 一个随手填的P控制
    double pkP = 6.5;
    double pkD = 0.2;
    double akP = 1.7;
    double akD = 0.1;
    // int delay = 10;
    int stability_counter = 0;
    double a_error = shortestDiff(Math::getDeg(odom.currentAngle), target_theta);
    double x_error = target_x - odom.globalPoint.x;
    double y_error = target_y - odom.globalPoint.y;
    double a_error_previous;
    double x_error_previous;
    double y_error_previous;
    double a_error_derivative;
    double x_error_derivative;
    double y_error_derivative;
    double unadjusted_x_input;
    double unadjusted_y_input;
    double unadjusted_a_input;
    double xy_error_previous;
    double xy_error_derivative;
    timer Timer;
    Timer.clear();
    double totaltime = 0;
    // double total_error = abs(a_error) + abs(x_error) + abs(y_error);
    double xy_error = sqrt(x_error * x_error + y_error * y_error);
    while (stability_counter < 15 && (totaltime = Timer.time(msec)) <= maxTimeMs)
    {
        a_error_previous = a_error;
        x_error_previous = x_error;
        y_error_previous = y_error;
        xy_error_previous = xy_error;
        a_error = shortestDiff(Math::getDeg(odom.currentAngle), target_theta);
        x_error = target_x - odom.globalPoint.x;
        y_error = target_y - odom.globalPoint.y;
        xy_error = sqrt(x_error * x_error + y_error * y_error);
        // total_error = fabs(a_error) + fabs(x_error) + fabs(y_error);

        a_error_derivative = (a_error - a_error_previous);
        x_error_derivative = (x_error - x_error_previous);
        y_error_derivative = (y_error - y_error_previous);
        xy_error_derivative = xy_error - xy_error_previous;

        if (xy_error < 0.9 &&  fabs(a_error) < 2)
            stability_counter++; // Counter to determine if the robot has settled
        else
            stability_counter = 0;
        // Adjustment to make path straight when not at 45 degrees at higher than max speed in one direction

        unadjusted_x_input = KeepInRange_abs(pkP * x_error + pkD * xy_error_derivative * (x_error / xy_error), 12, 73);
        unadjusted_y_input = KeepInRange_abs(pkP * y_error + pkD * xy_error_derivative * (y_error / xy_error), 12, 73);
        unadjusted_a_input = akP * a_error + akD * a_error_derivative;

        // if(unadjusted_x_input > 1 || unadjusted_y_input > 1) input_scaler = std::max(unadjusted_x_input, unadjusted_y_input);
        // else input_scaler = 1;
        // x_input = unadjusted_x_input/input_scaler;
        // y_input = unadjusted_y_input/input_scaler;
        //fieldCentricVRUN(unadjusted_x_input, unadjusted_y_input, unadjusted_a_input, odom);
        task::sleep(10);
    }
    //fieldCentricVRUN(0, 0, 0, odom);
    //printf("===============end moveTo in %lfms=============", totaltime);
}
void PID::moveTo(double x, double y, int maxTimeMs, Odom& odom)
{
    double target_x = x;
    double target_y = y;              // Convert to inverted okapi y-direction
    //double target_theta = finalAngle; // Convert to default okapi clockwise angle

    //printf("odom : X: %lf, Y: %lf\n",odom.globalPoint.x,odom.globalPoint.y);

    // PD parameters:
    //! 一个随手填的P控制
    double pkP = 4;
    double pkD = 0.2;
    //double akP = 1.5;
    //double akD = 0.1;
    // int delay = 10;
    int stability_counter = 0;
    //double a_error = shortestDiff(Math::getDeg(odom.currentAngle), target_theta);
    double x_error = target_x - odom.globalPoint.x;
    double y_error = target_y - odom.globalPoint.y;
    //double a_error_previous;
    double x_error_previous;
    double y_error_previous;
    //double a_error_derivative;
    double x_error_derivative;
    double y_error_derivative;
    double unadjusted_x_input;
    double unadjusted_y_input;
    //double unadjusted_a_input;
    double xy_error_previous;
    double xy_error_derivative;
    timer Timer;
    Timer.clear();
    double totaltime = 0;
    // double total_error = abs(a_error) + abs(x_error) + abs(y_error);
    double xy_error = sqrt(x_error * x_error + y_error * y_error);
    while (stability_counter < 15 && (totaltime = Timer.time(msec)) <= maxTimeMs)
    {
        //a_error_previous = a_error;
        x_error_previous = x_error;
        y_error_previous = y_error;
        xy_error_previous = xy_error;
        //a_error = shortestDiff(Math::getDeg(odom.currentAngle), target_theta);
        //printf("currentAngle: %lf\n", Math::getDeg(odom.currentAngle));
        x_error = target_x - odom.globalPoint.x;
        y_error = target_y - odom.globalPoint.y;
        //printf("odom : X: %lf, Y: %lf\n",odom.globalPoint.x,odom.globalPoint.y);
        xy_error = sqrt(x_error * x_error + y_error * y_error);
        // total_error = fabs(a_error) + fabs(x_error) + fabs(y_error);

        //a_error_derivative = (a_error - a_error_previous);
        
        x_error_derivative = (x_error - x_error_previous);
        y_error_derivative = (y_error - y_error_previous);
        xy_error_derivative = xy_error - xy_error_previous;

        if (xy_error < 2)// && fabs(a_error) < 4)
            stability_counter++; // Counter to determine if the robot has settled
        else
            stability_counter = 0;
        // Adjustment to make path straight when not at 45 degrees at higher than max speed in one direction

        unadjusted_x_input = KeepInRange_abs(pkP * x_error + pkD * xy_error_derivative * (x_error / xy_error), 0, maxSpeed);
        unadjusted_y_input = KeepInRange_abs(pkP * y_error + pkD * xy_error_derivative * (y_error / xy_error), 0, maxSpeed);
        //unadjusted_a_input = akP * a_error + akD * a_error_derivative;

        fieldCentricVRUN(unadjusted_x_input * 0.5, unadjusted_y_input * 0.5, odom);//, unadjusted_a_input * 0.7, odom);
        task::sleep(10);
    }
    fieldCentricVRUN(0, 0, odom);
    //printf("===============end moveTo in %lfms=============", totaltime);
}

void PID::moveToQuick(double x, double y, double finalAngle,int maxTimeMs, Odom odom)
{
    double target_x = x;
    double target_y = y;              // Convert to inverted okapi y-direction
    double target_theta = finalAngle; // Convert to default okapi clockwise angle
    // PD parameters:
    //! 一个随手填的P控制
    double pkP = 4.5;
    double pkD = 0.2;
    double akP = 1.5;
    double akD = 0.1;
    // int delay = 10;
    int stability_counter = 0;
    double a_error = shortestDiff(Math::getDeg(odom.currentAngle), target_theta);
    double x_error = target_x - odom.globalPoint.x;
    double y_error = target_y - odom.globalPoint.y;
    double a_error_previous;
    double x_error_previous;
    double y_error_previous;
    double a_error_derivative;
    double x_error_derivative;
    double y_error_derivative;
    double unadjusted_x_input;
    double unadjusted_y_input;
    double unadjusted_a_input;
    double xy_error_previous;
    double xy_error_derivative;
    timer Timer;
    Timer.clear();
    double totaltime = 0;
    // double total_error = abs(a_error) + abs(x_error) + abs(y_error);
    double xy_error = sqrt(x_error * x_error + y_error * y_error);
    while (stability_counter < 2 && (totaltime = Timer.time(msec)) <= maxTimeMs)
    {
        a_error_previous = a_error;
        x_error_previous = x_error;
        y_error_previous = y_error;
        xy_error_previous = xy_error;
        a_error = shortestDiff(Math::getDeg(odom.currentAngle), target_theta);
        x_error = target_x - odom.globalPoint.x;
        y_error = target_y - odom.globalPoint.y;
        xy_error = sqrt(x_error * x_error + y_error * y_error);
        // total_error = fabs(a_error) + fabs(x_error) + fabs(y_error);

        a_error_derivative = (a_error - a_error_previous);
        
        x_error_derivative = (x_error - x_error_previous);
        y_error_derivative = (y_error - y_error_previous);
        xy_error_derivative = xy_error - xy_error_previous;

        if (xy_error < 1.3 && fabs(a_error) < 2)
            stability_counter++; // Counter to determine if the robot has settled
        else
            stability_counter = 0;
        // Adjustment to make path straight when not at 45 degrees at higher than max speed in one direction

        unadjusted_x_input = KeepInRange_abs(pkP * x_error + pkD * xy_error_derivative * (x_error / xy_error), 0, maxSpeed);
        unadjusted_y_input = KeepInRange_abs(pkP * y_error + pkD * xy_error_derivative * (y_error / xy_error), 0, maxSpeed);
        unadjusted_a_input = akP * a_error + akD * a_error_derivative;

    
        // if(unadjusted_x_input > 1 || unadjusted_y_input > 1) input_scaler = std::max(unadjusted_x_input, unadjusted_y_input);
        // else input_scaler = 1;
        // x_input = unadjusted_x_input/input_scaler;
        // y_input = unadjusted_y_input/input_scaler;
        //fieldCentricVRUN(unadjusted_x_input, unadjusted_y_input, unadjusted_a_input, odom);
        task::sleep(10);
    }
    //fieldCentricVRUN(0, 0, 0, odom);
    //printf("===============end moveTo in %lfms=============", totaltime);
}

void PID::rotateTo(double finalAngle, int maxTimeMs, Odom& odom)
{
    //printf("rotate : X : %lf, Y : %lf\n");
    //PID::moveTo(odom.globalPoint.x, odom.globalPoint.y, finalAngle, maxTimeMs, odom);

}
void PID::moveToXY(double x, double y, int maxTimeMs, Odom& odom)
{
    PID::moveTo(x, y, maxTimeMs, odom);
}
void PID::rotateToXY(double x, double y, int maxTimeMs, Odom& odom, double bias)
{
    double targetDeg = getAngle(x - odom.globalPoint.x, y - odom.globalPoint.y) + bias;
    printf("%lf", targetDeg);
    //PID::moveTo(odom.globalPoint.x, odom.globalPoint.y, targetDeg, maxTimeMs, odom);
}

void PID::reset()
{
    // Movement PID
    derivative_x = 0.0;
    derivative_y = 0.0;
    prevError_x = 0.0;
    prevError_y = 0.0;
    // Turn PID
    turn_derivative = 0.0;
    turn_prevError = 0.0;
    // Reset function - for debugging
    dist = 0.0;
    turn_e = 0.0;
}

double PID::getError(PIDtype type)
{
    if (type == PIDtype::movement)
    {
        return dist;
    }
    else
    {
        return turn_e;
    }
}
void PID::setMaxSpeed(double maxspeed)
{
    maxSpeed = maxspeed;
}
