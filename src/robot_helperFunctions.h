

#pragma once

#ifdef	__cplusplus
extern "C" {
#endif





#define local_PI 3.14159265359
#define local_PI_over_2 1.57079632679
#define local_PI_x_2 6.28318530718



#define deg2rad(x) ((x*local_PI)/180)




//////////////  FUNCTION DELCARATION  //////////////
int fSign(float x);
float fLimit(float x, float low, float high);
float fAbs(float x);
float getRadianAngleDif(float start, float finish);
float normalizeRadianAngle(float x);
float getTriangleDist(float mean, float cutoff);



#ifdef	__cplusplus
}
#endif