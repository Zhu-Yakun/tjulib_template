# pragma once
#include "vex.h"
#include <vector>
//using namespace tjulib;

namespace tjulib {
    class straightChassis {
    public:
        /**
         * @brief Construct a new straightChassis
         *
         * @param leftMotors 左侧电机序列
         * @param rightMotors 右侧电机序列
         */
        straightChassis(std::vector<vex::motor*>& leftMotors, std::vector<vex::motor*>& rightMotors);
        
        // 驱动函数
        void setSpinPct(double Lspeed, double Rspeed);
        void VRUN(double Lspeed, double Rspeed);
        
        // 刹车函数
        void setStop(vex::brakeType type);
        void setStopType(vex::brakeType type);
        
    //private:
        std::vector<vex::motor*> _leftMotors;
        std::vector<vex::motor*> _rightMotors;

        const int deadzone = 5;//设置死区

    /**
     * @brief 一个手柄控制线性运动，一个手柄控制转弯
     * @brief 没有套用循环，使用时请套用循环
     */
        void ArcadeDrive();

        /**
         * @brief 左操纵杆控制左侧， 右操纵杆控制右侧
         * @brief 没有套用循环，使用时请套用循环
         */
        void TankDrive();
    };
} // namespace tjulib