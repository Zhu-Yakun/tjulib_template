/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Miracle                                                   */
/*    Created:      2023/11/1 23:12:20                                        */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
// 如果进行技能赛就def，否则注释，进行自动
//#define SKILL

#include "vex.h"
#include "tjulib.h"
#include <string>

using namespace vex;
using namespace tjulib;

// 调参区域
/*************************************************************/
// 编码器offset参数 单位inches
double horizonalOffset = 1;
double verticalOffset  = 1;

// 轮周  单位inches 一般不需要动
double wheelCircumference = 2.75;

// 设置初始位置、角度
// 初始位置，单位为inches
double init_pos_x = 0;
double init_pos_y = 0;

// 逆时针角度，范围在0 ~ 360°之间
double init_angle = 0;

// 移动、转向pid，参数可自行调整
pidParams fwde(2.5, 0.2, 0.1, 5, 10), turn(2.2, 0.2, 0.2, 5, 5);
/*************************************************************/

//变量定义区
competition Competition;

std::vector<vex::motor*> _leftMotors = {&L1, &L2, &L3};
std::vector<vex::motor*> _rightMotors = {&R1, &R2, &R3};

Odom odom(horizonalOffset, verticalOffset, wheelCircumference, encoderVertical, encoderHorizonal, imu, true);  // 调参时改成false
pidControl fwdControl(&fwde), turnControl(&turn);
odomtryStraightChasiss drive(_leftMotors, _rightMotors, &odom, &fwdControl, &turnControl);

// odom线程
int a(){
    odom.Odometry(odom);
    return 0;
}

void pre_auton(){
    thread posTrack(a);
    while(!imu.installed())task::sleep(8);
    imu.calibrate();
    while(imu.isCalibrating())task::sleep(8);
    task::sleep(3000);

    imu.setRotation(360 - init_angle, deg);
    odom.reset();
    odom.setPosition(init_pos_x, init_pos_x, Math::getRadians(init_angle));

    encoderVertical.resetRotation();
    encoderHorizonal.resetRotation();
}

void autonomous()
{
    
}

void skillautonoumous()
{
    
}

void usercontrol()
{
    Controller1.ButtonA.pressed([](){
        
    });
    
    Controller1.ButtonR1.released([]() {        
        roller.stop(brakeType::coast); 
    });
    
    Controller1.ButtonR2.pressed([]() {
        static bool motorRunning = false; // 用于追踪电机状态
  
        if (!motorRunning) {
            roller.spin(forward,100,pct);
        } else {
           roller.stop();// 停止电机旋转
        }
        motorRunning = !motorRunning; // 切换电机状态}
    });
    
    while(true){
        drive.ArcadeDrive();
        // double currentAngle = imu.rotation();
        // while(currentAngle >= 360)
        //     currentAngle -= 360;
        // while(currentAngle < 0)
        //     currentAngle += 360;
        // currentAngle = 360 - currentAngle;
        // printf("imu : %lf\n", currentAngle);

        // 调试时通过按键进入自动
        // if(Controller1.ButtonX.pressing()){
        //     autonomous();
        // }
        // if(Controller1.ButtonY.pressing()){
        //     skillautonoumous();
        // }
    }
}


int main() {
  // Set up callbacks for autonomous and driver control periods.
  #ifdef SKILL
  Competition.autonomous(skillautonoumous);
  #else
    
    Competition.autonomous(autonomous);
    
  #endif


  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}

