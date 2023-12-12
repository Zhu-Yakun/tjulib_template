#pragma once
#include "vex.h"
#include "velPID.hpp"
#include "Math-Functions.h"
#include "tjulib/odom.hpp"

using namespace vex;
using namespace tjulib;

struct args{
    motor *pmotor;
    velPID* pMPID;
    float* ptargetRPM;

    args(motor *pmotor, velPID* pMPID, float* ptargetRPM):pmotor(pmotor), pMPID(pMPID), ptargetRPM(ptargetRPM){};
};
int processVelPID(void * nargs);

class PIDMotor: public motor{

protected :
    task spinPIDMotor;
    args arg;
    velPID MPID;
    float targetRPM;

    double minSpeed = 3;// 电机的最低速度
    double accelRate = 100;//加速率：改变加速/减速的速度
    double deaccelRate = 4;//减速时的速度倍率（末尾的斜率）。较高时开始放慢速度
    double targetRange = .25;
    

public:
    PIDMotor(int index, vex::gearSetting type=ratio18_1, bool reverse=false, float Kp=1, float Ki=1, float Kd=1);

    void setPID( float Kp, float Ki, float Kd);

    //暂时只有pct的
    void spinPID( directionType dir, float velocity); 

    void distancePID();

    void rotateDegrees(double degrees, int maxSpeed);

    void set_distance_pid_params(double AminSpeed, double AaccelRate, double AdeaccelRate, double AtargetRange);

    ~PIDMotor();
};



enum class PIDtype
{
  turn,
  movement
};

class PID
{
private:
  double kP, kD, derivative_x, derivative_y, prevError_x, prevError_y, kF;
  double turn_kP, turn_kD, turn_derivative, turn_prevError, turn_kF;
  double dist, turn_e;
  double maxSpeed=100;

public:
  // Constructs PID constants
  PID(double _kP, double _kD, double _turn_kP, double _turn_kD);
  void moveTo(double x, double y, int maxTimeMs, tjulib::Odom& odom);
  void moveToQuick(double x, double y, double finalAngle,int maxTimeMs, tjulib::Odom odom);

  //void moveTo(double x, double y, double finalAngle, int maxTimeMs);
  void moveToXY(double x, double y, int maxTimeMs, tjulib::Odom& odom);
  void small_Range_moveTo(double x, double y, double finalAngle, int maxTimeMs, tjulib::Odom odom);
  void rotateTo(double finalAngle, int maxTimeMs, tjulib::Odom& odom);
  void rotateToXY(double x, double y, int maxTimeMs, tjulib::Odom& odom, double bias = 0.0);
  
  void reset();
  double getError(PIDtype type);
  void setMaxSpeed(double maxspeed);
  // double keepInRange(double n, double range);
};






