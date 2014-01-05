
#include <math.h>
#include "simulation_main.h"
#include "robot_helperFunctions.h"
#include "robot_routePlanner.h"



float getAngleBetweenPoints(float x1, float y1, float x2, float y2){
    float y_delta = y2-y1;
    float x_delta = x2-x1;
    return atan2f(y_delta,x_delta);
}



float PI_controller(sp_PI_data sp_PI, float sp, float fb, int timeStamp)
{
    float error = sp - fb;
    float averageError = (sp_PI->lastError + error)/2;
    sp_PI->lastError = error;
    float deltaTime = timeStamp - sp_PI->lastTimeStamp;
    sp_PI->lastTimeStamp = timeStamp;
    
    float output = error*sp_PI->P + error*deltaTime*sp_PI->I;
    return output;
}





s_routePlanner_return gotoWaypoint(float x, float y, float x_goal, float y_goal, float angle)
{
    float toGoalAngle = getAngleBetweenPoints(x,y,x_goal,y_goal);
    float distance = sqrt(powf(x_goal-x,2) + powf(y_goal-y,2));
    float angleDelta = getRadianAngleDif(angle,toGoalAngle);
    s_routePlanner_return results;
    results.steeringCmd = fLimit(angleDelta*150,-100,100);
    results.finished = 0;
    results.power = __min(distance*20,2000/fAbs(results.steeringCmd));
    results.power = __min(results.power, 100);
    
    if(distance < .5)
    {
        results.power = 0;
        results.finished = 1;
    }
    
    return results;
    
    
}


//need to create an x and y equation that describes the car's path
//the function returns finished when the car reaches the mid point


//
//s_routePlanner_return gotoSplineWaypoint(s_splineRouteData s_spline, float x, float y, float angle)
//{
//    //float toGoalAngle = getAngleBetweenPoints(x,y,x_goal,y_goal);
//    //float distance = sqrt(powf(x_goal-x,2) + powf(y_goal-y,2));
//    float angleDelta = getRadianAngleDif(angle,toGoalAngle);
//    //sp_simulation->debug = toGoalAngle;
//    s_routePlanner_return results;
//    results.steeringCmd = fLimit(angleDelta*55,-100,100);
//    results.finished = 0;
//    results.power = __min(distance*20,2000/fAbs(results.steeringCmd));
//    results.power = __min(results.power, 100);
//    
//    
//    if(distance < .5)
//    {
//        results.power = 0;
//        results.finished = 1;
//    }
//    
//    return results;
//    
//    
//}



