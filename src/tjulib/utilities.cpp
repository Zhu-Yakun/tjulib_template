#include "vex.h"
#include "tjulib/Math-Functions.h"
#include "utilities.h"
using namespace vex;
#define PI 3.14159265358979323846
double warpAngle_deg(double angle)
{
    angle = angle - int(angle) / 360 * 360;
    angle = angle > 0 ? angle : angle + 360;
    return angle;
}

// 方向cur、tar[0,360)，将cur方向移动到tar方向，
// 就近选择移动方式，顺时针为正逆时针为负，返回值的大小为需要旋转的度数
double shortestDiff(double cur, double tar)
{
    tar = warpAngle_deg(tar);
    cur = warpAngle_deg(cur);
    double t = tar - cur; // 假设顺时针转，夹角为t
    if (t < 0)            // 如果t<0，说明顺时针转的历程中需要跨越临界点(经过角度0)
        t = t + 360;      // 修正为正值
    if (t > 180)          // 如果顺时针转180度以上，则改为逆时针转360-t度
        t = -(360 - t);
    return t;
}

// 获得从机器人当前位置到任何位置的角度
double getAngle(double x, double y)
{
    // atan2是一个函数，在C语言里返回的是指方位角，C 语言中atan2的函数原型为 double atan2(double y, double x) ，返回以弧度表示的 y/x 的反正切,(-pai,pai]。
    // if(fabs(x)<1e-4&&fabs(y)<1e-4)
    //   return 0; //0向量错误
    return warpAngle_deg(atan2(x, y) * 180.0 / PI);
}
double KeepInRange(double x, const double min, const double max)
{
    if (x > max)
        return max;
    if (x < min)
        return min;
    return x;
}
double KeepInRange_abs(double x, const double min, const double max)
{
    int sign = x > 0 ? 1 : -1;
    double fabsx = fabs(x);
    if (fabsx > max)
        fabsx = max;
    if (fabsx < min)
        fabsx = min;

    return fabsx * sign;
}
