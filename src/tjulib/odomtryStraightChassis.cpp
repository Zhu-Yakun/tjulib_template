#include "vex.h"
#include "tjulib/chassis/odomtryStraightChassis.hpp"
#include "tjulib/Math-Functions.h"

namespace tjulib
{
    T odomtryStraightChasiss::angleWrap(T deg)
    {
        double robotAngle = odom->globalPoint.angle * toDegrees;
        while (deg > robotAngle + 180) // Subtract 360 if angle is too large
        {
            deg -= 360; // Equivalent angle but subtract a rotation
        }
        while (deg <= robotAngle - 180) // Add 360 if angle is too large
        {
            deg += 360; // Equivalent angle but add a rotation
        }
        
        return deg; // Return the new closest angle
    }

    T odomtryStraightChasiss::getDistanceTo(Point target)
    {
        return sqrt(pow(odom->globalPoint.x - target.x, 2) + pow(odom->globalPoint.y - target.y, 2) * 1.0);
    }

    T odomtryStraightChasiss::getDegreeTo(Point target)
    {
        T relativeX = target.x - odom->globalPoint.x;
        T relativeY = target.y - odom->globalPoint.y;

        T deg = toDegrees * atan2(relativeY, relativeX);
        
        if((relativeX > 0 && relativeY > 0) || (relativeX < 0 && relativeY > 0))
            deg += 0;
        else if((relativeX > 0 && relativeY < 0) || (relativeX < 0 && relativeY < 0))
            deg += 360;
        else
            ;
        return deg;
    }

    double getWrap2pi(double currentAngle){
        while(currentAngle >= 360)
            currentAngle -= 360;
        while(currentAngle < 0)
            currentAngle += 360;
        return currentAngle;
    }

    #define DELTA_ANGLE_EPO 0.5

    void odomtryStraightChasiss::turnToTarget(Point target, T maxSpeed, double maxtime_ms, int fwd)
    {
        turnToAngle(getDegreeTo(target),maxSpeed, maxtime_ms, fwd);
    }

    void odomtryStraightChasiss::turnToAngle(double angle, T maxSpeed, double maxtime_ms, int fwd){
        timer mytime;
        mytime.clear();
        double totaltime = 0;
        T finalTurnSpeed = 3;

        T targetDeg = getWrap2pi(angle); // Obtain the closest angle to the target position
        
        double currentAngle = -imu.rotation();
        double prev_speed = finalTurnSpeed;
        int init =0;

        while ((std::abs(360 - getWrap2pi(-currentAngle)-targetDeg) > 2 || odom->deltaAngle < DELTA_ANGLE_EPO) && (totaltime=mytime.time(msec)<maxtime_ms)) // If within acceptable distance, PID output is zero.
        {
            currentAngle = -imu.rotation();
            if(!fwd)
                targetDeg += 180;
            while(targetDeg >= 360)
                targetDeg -= 360;    
            
            if(currentAngle > 0){
                while(targetDeg < currentAngle){
                    if(targetDeg + 360 < currentAngle)
                        targetDeg += 360;
                    else{
                        if(currentAngle - targetDeg < 360 + targetDeg - currentAngle)
                            break;
                        else{
                            targetDeg += 360;
                            break;
                        }
                    }
                }
            }
            else{
                while(targetDeg > currentAngle){
                    if(targetDeg - 360 > currentAngle)
                        targetDeg -= 360;
                    else{
                        if(targetDeg - currentAngle < currentAngle + 360 - targetDeg)
                            break;
                        else{
                            targetDeg -= 360;
                            break;
                        }
                    }
                }
            }
            finalTurnSpeed = turnControl->pidCalcu(targetDeg, maxSpeed, currentAngle); // Plug angle into turning PID and get the resultant speed
            //判断是否减速
            if(finalTurnSpeed*prev_speed<0&& init > 0){
                maxSpeed *= 0.1;
            }
            init = 1;
            //置上次
            prev_speed = finalTurnSpeed;
            // if(anti_clock)
            //     finalTurnSpeed = -finalTurnSpeed;
            VRUN(-finalTurnSpeed, finalTurnSpeed);
            task::sleep(5);
        }
        setStop(brakeType::brake);
    }


    void odomtryStraightChasiss::moveToTarget(Point target, T maxFwdSpeed, T maxTurnSpeed, double maxtime_ms, int fwd)
    {
        timer mytime;
        mytime.clear();
        double totaltime = 0;
        
        turnToTarget(target, maxTurnSpeed, maxtime_ms / 2, fwd);

        T finalFwdSpeed = 3;

        T targetDistant;
        while (std::abs(finalFwdSpeed) > 2 &&(totaltime=mytime.time(msec)<maxtime_ms)) // If within acceptable distance, PID output is zero.
        {
            targetDistant = getDistanceTo(target); // Obtain the closest angle to the target position

            finalFwdSpeed = fwdControl->pidCalcu(targetDistant, maxFwdSpeed); // Plug angle into turning PID and get the resultant speed
            if(!finalFwdSpeed)
                finalFwdSpeed = 2;
            // Turn in place towards the position
            if(!fwd)
                finalFwdSpeed = -finalFwdSpeed;
            VRUN(finalFwdSpeed, finalFwdSpeed);

            task::sleep(5);
        }
        setStop(brakeType::brake);
    }
}