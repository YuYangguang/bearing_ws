
#ifndef _MATHCAL_H__
#define _MATHCAL_H__

#include "core.hpp"
#include <math.h>

#define PI			3.141592656
// 角度转换为弧度
#define DEG2RAD		0.0174532925
// 弧度转换为角度
#define RAD2DEG		57.295779469

typedef struct _point
{
        double x;
        double y;
}Point;


F32 Sign233(F32 x);
F32 Fsg(F32 x, F32 d);
F32 Fhan(F32 x1, F32 x2, F32 r, F32 h);
F32 ValueLimit2(F32 value);
F32 ValueLimit(F32 value,F32 max,F32 min);
double degree2rad(double deg);
double rad2degree(double rad);
double regulateYaw(double yaw);

double TwoPointsCourseAngle(double P1Lo, double P1La, double P2Lo, double P2La);
double TwoPointsHorDistance(double cLon, double cLat, double aLon, double aLat);
Point ConvertLL2XY(double refLon, double refLat, double cLon, double cLat);



#endif
