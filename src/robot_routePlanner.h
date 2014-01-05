#pragma once

#ifdef	__cplusplus
extern "C" {
#endif



typedef struct{
    float output;
    float lastError;
    float P;
    float I;
    int lastTimeStamp;
}s_PI_data, *sp_PI_data;

typedef struct{
    float power;
    float steeringCmd;
    int finished;
}s_routePlanner_return;



typedef struct{
    float x_1, y_1, startingAngle;
    float x_2, y_2;
    float x_3, y_3;
}s_splineRouteData;


float getAngleBetweenPoints(float x1, float y1, float x2, float y2);
float PI_controller(sp_PI_data sp_PI, float sp, float fb, int timeStamp);
s_routePlanner_return gotoWaypoint(float x, float y, float x_goal, float y_goal, float angle);



#ifdef	__cplusplus
}
#endif