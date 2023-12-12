#define _USE_MATH_DEFINES
#include "vex.h"
#include <cmath>

#include <iostream>
#include <random>
#include <chrono>
#include "tjulib/odom.hpp"

#define PI 3.1415926

using namespace tjulib;

typedef float T;

extern std::stringstream data_to_sdcard;

vex::timer Time;

//构造函数
Odom::Odom(float hOffset, float vOffset, float wCircum, encoder& encoderVertical, encoder& encoderHorizonal, inertial& imu, bool isKalmanFilter):
    horizonalOffset(hOffset), verticalOffset(vOffset), wheelCircumference(wCircum), isKalmanFilter(isKalmanFilter),
    encoderVertical(encoderVertical), encoderHorizonal(encoderHorizonal), imu(imu), OMath(wCircum) {
    globalPoint = {0, 0, 0};
    prevGlobalPoint = {0, 0, 0};
    globalDeltaPoint = {0, 0, 0};

    //LOCAL COORDINATES
    localDeltaPoint = {0, 0};

    //SENSOR VALUES
    //encoder
    encoderVal = {0, 0}; //verticalEnc, horizonalEnc, backEnc
    prevEncoderVal = {0, 0};
    deltaEncoderVal = {0, 0};
    //angle
    currentAngle = 0.0;
    prevAngle = 0.0;
    deltaAngle = 0.0;
    //Math
};

//ODOMETRY FUNCTIONS

//setx0
void Odom::setx0(const T& x, const T&y){
  ekfilter.setx0(x,y);
}



//use filter rewrite needed
void Odom::updateSensors(){
  encoderVal.vertical = -OMath.degToInch(encoderVertical.rotation(deg)); //verticalE
  encoderVal.horizonal = OMath.degToInch(encoderHorizonal.rotation(deg)); //horizonalE

  deltaEncoderVal.vertical = encoderVal.vertical - prevEncoderVal.vertical; //verticalE
  deltaEncoderVal.horizonal = encoderVal.horizonal - prevEncoderVal.horizonal; //horizonalE

  prevEncoderVal.vertical = encoderVal.vertical; //verticalE
  prevEncoderVal.horizonal = encoderVal.horizonal; //horizonalE

  currentAngle = OMath.getRadians(imu.rotation());

  while(currentAngle >= 2*PI)
    currentAngle -= 2 * PI;
  while(currentAngle < 0)
    currentAngle += 2 * PI;
  currentAngle = 2 * PI - currentAngle;
  
  // 这里添加了一个-号是因为后面计算中角度是顺时针变化，现在用的是逆时针变化
  deltaAngle = -(currentAngle - prevAngle);
  prevAngle = currentAngle;
}



// ekf

T prevtheta = 0;
T theta1, deltaTheta=0.0;
T V = 0.0; 

void Odom::filteringData(){
  Eigen::Vector2d u;
  Eigen::Vector3d z;
  u << V, deltaTheta;
  //更新速度
  T dx=globalPoint.x-ekfilter.getState()(0),
         dy=globalPoint.y-ekfilter.getState()(1);
  V = sqrt(dx*dx+dy*dy);
  theta1 = atan2(dy,dx);
  deltaTheta = theta1-prevtheta;
  ekfilter.predict(u);
  z<<globalPoint.x,globalPoint.y,theta1;//当前的
  ekfilter.update(z);

  //预估完成
  Eigen::Vector3d xt = ekfilter.getState();
  globalPoint.x = xt(0);
  globalPoint.y = xt(1);
}



void Odom::updatePosition(){
 //Polar coordinates
    T deltaX = deltaEncoderVal.horizonal;
    T deltaY = deltaEncoderVal.vertical;
    T localX = 0;
    T localY = 0;

    if (deltaAngle == 0)
    { // prevent divide by 0
        localX = deltaX;
        localY = deltaY;
    }
    else
    {
        localX = 2 * sin(deltaAngle / 2) * (deltaX / deltaAngle + horizonalOffset);
        localY = 2 * sin(deltaAngle / 2) * (deltaY / deltaAngle + verticalOffset);
    }
    T  global_angle = prevAngle + deltaAngle/2 - PI/4;

    //Cartesian coordinates
    globalDeltaPoint.x = (localY * sin(global_angle)) + (localX * cos(global_angle)); 
    globalDeltaPoint.y = -((localY * cos(global_angle)) - (localX * sin(global_angle)));
    globalDeltaPoint.angle = deltaAngle;

    globalPoint.x = globalDeltaPoint.x + prevGlobalPoint.x;
    globalPoint.y = globalDeltaPoint.y + prevGlobalPoint.y;
    globalPoint.angle = currentAngle;

    prevGlobalPoint.x = globalPoint.x;
    prevGlobalPoint.y = globalPoint.y;
    
    if(isKalmanFilter){
        filteringData();
    }

    return;
}

void Odom::reset(){
  encoderVertical.resetRotation(); 
  encoderHorizonal.resetRotation(); 
  prevEncoderVal.vertical = 0.0; prevEncoderVal.horizonal = 0.0; 
  prevAngle = 0.0;
  prevGlobalPoint.x = 0.0; prevGlobalPoint.y = 0.0;
}

//都要使用进行初始化
void Odom::setPosition(T newX, T newY, T newAngle){
  reset();
  prevAngle = newAngle;
  prevGlobalPoint.x = newX;
  prevGlobalPoint.y = newY;
  globalPoint.angle = newAngle;
  globalPoint.x = newX;
  globalPoint.y = newY;
  ekfilter.setx0(newX, newY);
}

//ODOMETRY THREAD
int Odom::Odometry(Odom& odom){
  static T time_last = 0;
  T time_cur = 0;
  T deltime = 0;
  while(true) { 
    
    time_cur = Time.time(msec);
    deltime = time_cur-time_last;
    time_last=time_cur;
    odom.updateSensors();
    odom.updatePosition();
    printf("X: %lf, Y: %lf\n",odom.globalPoint.x,odom.globalPoint.y);
    //printf("V: %lf, H: %lf\n", Odom::encoderVal.vertical, Odom::encoderVal.horizonal);

    //Brain.Screen.printAt(1, 20, "X: %lf", Odom::globalPoint.x);
    //Brain.Screen.printAt(1, 40, "Y: %lf", Odom::globalPoint.y);
    
    //Brain.Screen.printAt(1, 120, "V: %lf", Odom::encoderVal.vertical);
    //Brain.Screen.printAt(1, 140, "H: %lf", Odom::encoderVal.horizonal);

    this_thread::sleep_for(10);
  }
  return 0;
}