#pragma once
#include"filter/ekf.hpp"
#include "Math-Functions.h"
#include "vex.h"

namespace tjulib{
    
    using namespace vex;

    struct Point {
        float x, y, angle;
    };

    struct encoderType{
        float vertical, horizonal;
    };

    class Odom {
    private:
        //CONSTANTS
        // 轮距
        const float horizonalOffset; //inches
        const float verticalOffset;
        //轮子
        const float wheelCircumference; //inches

        bool isKalmanFilter;

        encoder& encoderVertical;
        encoder& encoderHorizonal;
        inertial& imu;

        //滤波器
        ExtendedKalmanFilter ekfilter;

        //Math
        Math OMath;
    public: 
        Odom(float hOffset, float vOffset, float wCircum, encoder& encoderVertical, encoder& encoderHorizonal, inertial& imu, bool isKalmanFilter = false);
        //GLOBAL COORDINATES
        //全局坐标系下的当前位置，包括 x 和 y 坐标以及朝向角度。
        Point globalPoint; //x, y, angle
        //上一个时间步的全局坐标系下的位置
        Point prevGlobalPoint; 
        //从上一个时间步到当前时间步在全局坐标系下移动的距离和旋转的角度
        Point globalDeltaPoint; //change in x, change in y, change in angle

        //LOCAL COORDINATES
        //从上一个时间步到当前时间步在局部坐标系下移动的距离
        Point localDeltaPoint; //change in x, change in y

        //SENSOR VALUES
        //encoder
        encoderType encoderVal; //verticalEnc, horizonalEnc, backEnc
        encoderType prevEncoderVal; //prev verticalEnc, horizonalEnc, backEnc
        encoderType deltaEncoderVal; //change in verticalEnc, horizonalEnc, backEnc
        //angle
        float currentAngle;
        float prevAngle;
        float deltaAngle; //rad

        //ODOMETRY FUNCTIONS
        void updateSensors();
        void updatePosition();
        void filteringData();

        //将机器人的位置和传感器值重置为初始状态
        void reset();
        void setPosition(float newX, float newY, float newAngle);

        //ODOMETRY THREAD
        int Odometry(Odom& odom);
        void setx0(const float& x, const float&y);
    };
};

