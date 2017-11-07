
#include "smarteye/core/MathCal.h"
#include "smarteye/core/core.hpp"
#include <math.h>



F32 Sign233(F32 x)
{
    if(x>0)
        return 1;
    else if(x<0)
        return -1;
    else
        return 0;
}

F32 Fsg(F32 x, F32 d)
{
    F32 temp1;
    F32 temp2;
    temp1 = Sign233(x + d);
    temp2 = Sign233(x - d);
    return (0.5 * (temp1 - temp2));

}

F32 Fhan(F32 x1, F32 x2, F32 r, F32 h)
{
    F32 d;
    F32 a, a0, a1, a2;
    F32 y;
    F32 output;
    d = r * h * h;
    a0 = h * x2;
    y = x1 + a0;
    a1 = sqrt(d * (d + 8.0 * fabs(y)));
    a2 = a0 + Sign233(y) * (a1 - d) / 2.0;
    a = (a0 + y) * Fsg(y, d) + a2 * (1.0 - Fsg(y, d));
    output = -r * (a / d) * Fsg(a, d) - r * Sign233(a) * (1.0 - Fsg(a, d));

    return output;
}

F32 ValueLimit2(F32 value)
{
    S32 temp;
    temp = value * 1000;
    value = (F32)temp / 1000.0;
    return value;
}

F32 ValueLimit(F32 value,F32 max,F32 min)
{
    if(value > max)
        return max;
    else if(value < min)
        return min;
    else
        return value;
}

double degree2rad(double deg)
{
    double rad;
    rad=deg/180*M_PI;
    return rad;
}

/*将偏航角限定在-180度到180度之间*/
double regulateYaw(double yaw)
{
    while(yaw >360)
    {
        yaw = yaw-360;
    }
    while(yaw<-360)
    {
        yaw = yaw+360;
    }
    if(yaw > 180)
    {
        yaw =-(360-yaw);
    }
    if(yaw<-180)
    {
        yaw=360+yaw;
    }
    return(yaw);
}

double rad2degree(double rad)
{
    double degree;
    degree=rad/M_PI*180;
    return degree;
}

// Algorithm designed by Xiangke Wang
double TwoPointsCourseAngle(double P1Lo, double P1La, double P2Lo, double P2La)
{
    double temp_n=0;
    double temp_e=0;
    double angle=0;
    double Len=0;
    temp_n= P2La - P1La;
    temp_e=(P2Lo - P1Lo)*cos((double)P1La*DEG2RAD);
    Len = sqrt(temp_n*temp_n+temp_e*temp_e);

    // 计算航向角
    angle=0;
    if(Len> 0)
    {
        angle = acos((temp_n/Len));  // 与N轴的夹角
        if(temp_e>=0)
        {
            angle =angle *RAD2DEG;
        }
        else
        {
            angle = 360.0-angle * RAD2DEG;
        }
    }
    if (angle >= 360.0)
    {
        angle=angle - 360.0;
    }
    else if (angle < 0)
    {
        angle=angle + 360.0;
    }


    // 限制在0到360内
    if(angle < 0)
        angle = 0;
    else if(angle > 360.0)
        angle = 360.0;
    return angle;
}

// Algorithm designed by Xiangke Wang
// 计算两点水平距离
double TwoPointsHorDistance(double cLon, double cLat, double aLon, double aLat)
{
    double temp=0;
    double temp2=0;
    double Cclat=0;
    double Sclat=0;
    double Calat=0;
    double Salat=0;
    double Ca_c=0;
    double R = 6371004;
    Ca_c = cos(((double)aLon - (double)cLon)*DEG2RAD);
    Cclat = cos((double)cLat*DEG2RAD);
    Sclat = sin((double)cLat*DEG2RAD);
    Calat = cos((double)aLat*DEG2RAD);
    Salat = sin((double)aLat*DEG2RAD);
    temp2=Calat*Cclat*Ca_c + Salat*Sclat;
    if(temp2<-1)
        temp2=-1;
    if(temp2>1)
        temp2=1;
    temp = R * acos(temp2);
    return temp;
}

// Algorithm designed by Xiangke Wang
// 将经纬度转换为XY坐标
Point ConvertLL2XY(double refLon, double refLat, double cLon, double cLat)
{
    Point pos;
    double angle=0;
    double dis=0;
    angle = TwoPointsCourseAngle(refLon,refLat,cLon,cLat);
    dis=TwoPointsHorDistance(refLon,refLat,cLon,cLat);
    pos.x=dis*cos(angle*DEG2RAD);
    pos.y=dis*sin(angle*DEG2RAD);
    return pos;
}







