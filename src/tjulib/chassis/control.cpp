#include "tjulib/chassis/control.hpp"
using namespace tjulib;

// 应用两次正弦函数映射（sine mapping）来改变转向输入，让机器人在低速转向时更容易精确操控
double Control::_turnRemapping(double iturn)
{
    double denominator = sin(PI / 2 * CD_TURN_NONLINEARITY);
    double firstRemapIteration = sin(PI / 2 * CD_TURN_NONLINEARITY * iturn) / denominator;
    return sin(PI / 2 * CD_TURN_NONLINEARITY * firstRemapIteration) / denominator;
}

// 更新两个累加器变量
void Control::_updateAccumulators()
{
    if (negInertiaAccumulator > 1)
    {
        negInertiaAccumulator -= 1;
    }
    else if (negInertiaAccumulator < -1)
    {
        negInertiaAccumulator += 1;
    }
    else
    {
        negInertiaAccumulator = 0;
    }

    if (quickStopAccumulator > 1)
    {
        quickStopAccumulator -= 1;
    }
    else if (quickStopAccumulator < -1)
    {
        quickStopAccumulator += 1;
    }
    else
    {
        quickStopAccumulator = 0.0;
    }
}

// Cheesy Drive 控制函数
void Control::cheesyDrive(double ithrottle, double iturn)
{
    bool turnInPlace = false;
    double linearCmd = ithrottle;

    // 死区的处理
    if (fabs(ithrottle) < DRIVE_DEADBAND && fabs(iturn) > DRIVE_DEADBAND)
    {
        linearCmd = 0.0;
        turnInPlace = true;
    }
    else if (ithrottle - prevThrottle > DRIVE_SLEW)
    {
        linearCmd = prevThrottle + DRIVE_SLEW;
    }
    else if (ithrottle - prevThrottle < -(DRIVE_SLEW * 2))
    {
        linearCmd = prevThrottle - (DRIVE_SLEW * 2);
    }

    double remappedTurn = _turnRemapping(iturn);
    double left, right;

    if (turnInPlace)
    {
        left = remappedTurn * std::abs(remappedTurn);
        right = -remappedTurn * std::abs(remappedTurn);
    }
    else
    {
        double negInertiaPower = (iturn - prevTurn) * CD_NEG_INERTIA_SCALAR;
        negInertiaAccumulator += negInertiaPower;
        double angularCmd = std::abs(linearCmd) * (remappedTurn + negInertiaAccumulator) * CD_SENSITIVITY - quickStopAccumulator;
        right = left = linearCmd;
        left += angularCmd;
        right -= angularCmd;
        _updateAccumulators();
    }

    prevTurn = iturn;
    prevThrottle = ithrottle;

    // 控制底盘运动
    setSpinPct(left, right);
}
