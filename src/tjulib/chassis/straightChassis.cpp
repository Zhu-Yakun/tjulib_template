#include "vex.h"
#include "tjulib/chassis/straightChassis.hpp"


using namespace tjulib;

typedef double T;

// DRIVE HELPER FUNCTIONS
straightChassis::straightChassis(std::vector<vex::motor*>& leftMotors, std::vector<vex::motor*>& rightMotors)
    : _leftMotors(leftMotors), _rightMotors(rightMotors)
{
}

void straightChassis::setSpinPct(T Lspeed, T Rspeed)
{
    for (vex::motor* motor : (_leftMotors))
    {
        motor->spin(vex::directionType::fwd, Lspeed, vex::pct);
    }
    for (vex::motor* motor : (_rightMotors))
    {
        motor->spin(vex::directionType::fwd, Rspeed, vex::pct);
    }
}

void straightChassis::VRUN(T Lspeed, T Rspeed)
{
    for (vex::motor *motor : (_leftMotors))
    {
        vexMotorVoltageSet(motor->index(), Lspeed * 12000.0 / 100.0);
    }
    for (vex::motor *motor : (_rightMotors))
    {
        vexMotorVoltageSet(motor->index(), Rspeed * 12000.0 / 100.0);
    }
}

void straightChassis::setStop(vex::brakeType type)
{
    for (vex::motor *motor : (_leftMotors))
    {
        motor->stop(type);
    }
    for (vex::motor *motor : (_rightMotors))
    {
        motor->stop(type);
    }
}

void straightChassis::setStopType(vex::brakeType type)
{
    for (vex::motor *motor : (_leftMotors))
    {
        motor->setStopping(type);
    }
    for (vex::motor *motor : (_rightMotors))
    {
        motor->setStopping(type);
    }
}


void straightChassis::ArcadeDrive()
{
    // Retrieve the necessary joystick values
    int leftY = Controller1.Axis3.position(percent);
    int rightX = Controller1.Axis1.position(percent);
    if (abs(leftY) < deadzone)
    {
        leftY = 0;
    }
    if (abs(rightX) < deadzone)
    {
        rightX = 0;
    }
    // 限速
    //leftY = leftY * 0.5;
    rightX = rightX * 0.5;

    for (vex::motor *motor : (this->_leftMotors))
    {
        if(leftY ==0 && rightX == 0){
            motor->stop(brake);
        }
        motor->spin(vex::directionType::fwd, leftY + rightX, vex::pct);
    }
    for (vex::motor *motor : (this->_rightMotors))
    {
        if(leftY == 0 && rightX == 0){
            motor->stop(brake);
        }
        motor->spin(vex::directionType::fwd, leftY - rightX, vex::pct);
    }
}

void straightChassis::TankDrive()
{
    // Retrieve the necessary joystick values
    int leftY = Controller1.Axis3.position(percent);
    int rightX = Controller1.Axis2.position(percent);
    if (abs(leftY) < deadzone)
    {
        leftY = 0;
    }
    if (abs(rightX) < deadzone)
    {
        rightX = 0;
    }
    for (vex::motor *motor : (this->_leftMotors))
    {
        motor->spin(vex::directionType::fwd, leftY, vex::pct);
    }
    for (vex::motor *motor : (this->_rightMotors))
    {
        motor->spin(vex::directionType::fwd, rightX, vex::pct);
    }
}