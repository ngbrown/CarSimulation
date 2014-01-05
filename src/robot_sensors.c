#ifdef	__cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "simulation_main.h"
#include "robot_physicalParams.h"
#ifdef SIMULATION_ACTIVE
#include "simulation_engine.h"
//sp_carSimulation sp_simulation;
#endif




#include "robot_sensors.h"



#ifdef SIMULATION_ACTIVE
sp_robot_sensors UpdateSensors(){
    static int initializationComplete = 0;
    static sp_robot_sensors sp_sensors;
    if(!initializationComplete)
    {
        initializationComplete = 1;
        sp_sensors = (sp_robot_sensors)malloc(sizeof(s_robot_sensors));
    }
    
    sp_sensors->compass = sp_simulation->sensorSim->compassAngle;
    sp_sensors->encoder_left = sp_simulation->sensorSim->EncoderTicks_Left;
    sp_sensors->encoder_right = sp_simulation->sensorSim->EncoderTicks_Right;
    sp_sensors->gps_x = sp_simulation->sensorSim->gps_x;
    sp_sensors->gps_y = sp_simulation->sensorSim->gps_y;
    sp_sensors->gps_new = sp_simulation->sensorSim->gpsLastUpdateTime == sp_simulation->milisecondsOfLastUpdate;

    return sp_sensors;
}
#endif





s_encoder_decode_return encoder_decode(int newLeftCount, int newRightCount, sp_carPhysicalParameters sp_phys)
{
    static int initializationComplete= 0;
    static int lastLeftCount = 0;
    static int lastRightCount = 0;
    static float angleConstant = 0;
    static float distConstant = 0;
    static sp_carPhysicalParameters sp_carPhysics;
    
    
    if(!initializationComplete)
    {
        initializationComplete = 1;
        lastLeftCount = newLeftCount;
        lastRightCount = newRightCount;
        angleConstant = local_PI_x_2*sp_phys->encoderWheelRadius/(sp_phys->encoderWidth*sp_phys->encoderTicksPerRev);
        distConstant = local_PI_x_2*sp_phys->encoderWheelRadius/sp_phys->encoderTicksPerRev;
        sp_carPhysics = sp_phys;
    }
    int leftDelta = newLeftCount-lastLeftCount;
    int rightdelta = newRightCount-lastRightCount;
    float angleDelta = angleConstant*(rightdelta-leftDelta);
    
    float delta_right;
    float delta_forward;
    float distance = (leftDelta + rightdelta)*distConstant/2.0;
    
    if(rightdelta-leftDelta != 0)
    {
        float radius = fAbs(distance/angleDelta);
        if(distance > 0)
        {
            if(angleDelta > 0)
            {
                delta_forward = radius*sinf(angleDelta); //turning left, going forward
                delta_right = radius*(1-cosf(angleDelta)); 
            }else{
                delta_forward = radius*sinf(-1*angleDelta); //turning right, going forward
                delta_right = radius*(cosf(angleDelta)-1); 
            }
        }else{
            if(angleDelta > 0)
            {
                delta_forward = radius*sinf(-1*angleDelta); //turning left, going back
                delta_right = radius*(1-cosf(angleDelta)); 
            }else{
                delta_forward = radius*sinf(angleDelta); //turning right, going back
                delta_right = radius*(cosf(angleDelta)-1); 
            }
        }
            
    }else{
        delta_right = 0;
        delta_forward = distance;
    }
    
    lastLeftCount = newLeftCount;
    lastRightCount = newRightCount;
    
    s_encoder_decode_return returnStruct;
    returnStruct.delta_right = delta_right;
    returnStruct.delta_forward = delta_forward;
    returnStruct.delta_angle = angleDelta;

    return returnStruct;
}







s_fusion_return sensorFusionAndMapping(){
    static int initializationComplete = 0;
    static sp_carPhysicalParameters sp_carPhysics;
    static float estimated_angle = 0;
    static float estimated_x = 0;
    static float estimated_y = 0;
    
    if(!initializationComplete)
    {
        initializationComplete = 1;
        sp_carPhysics = initializeCarPhy();
    }
    
    sp_robot_sensors sp_sensorResults = UpdateSensors();
    
    
    s_encoder_decode_return encoderInfo = encoder_decode(sp_sensorResults->encoder_left,sp_sensorResults->encoder_right,sp_carPhysics);
    float compassHeading = sp_sensorResults->compass;
    float gps_x = sp_sensorResults->gps_x;
    float gps_y = sp_sensorResults->gps_y;
    int gps_new = sp_sensorResults->gps_new;
    
    float delta = encoderInfo.delta_angle * 1;
    estimated_angle += delta;
    delta = getRadianAngleDif(estimated_angle,compassHeading)*.005;
    estimated_angle += delta;
    //sp_simulation->debug = delta;
    //gps_new = 0;
    delta = encoderInfo.delta_forward*cos(estimated_angle) + encoderInfo.delta_right*sin(estimated_angle);
    if(gps_new)
    {
        delta += (gps_x-estimated_x)*.05;
    }
    estimated_x += delta;
    
    delta = encoderInfo.delta_forward*cos(estimated_angle-local_PI_over_2) + encoderInfo.delta_right*sin(estimated_angle-local_PI_over_2);
    if(gps_new)
    {
        delta += (gps_y-estimated_y)*.05;
    }
    estimated_y += delta;
    
    estimated_angle = normalizeRadianAngle(estimated_angle);
    
    s_fusion_return returnStruct;
    returnStruct.x = estimated_x;
    returnStruct.y = estimated_y;
    returnStruct.angle = estimated_angle;
    return returnStruct;
}



#ifdef	__cplusplus
}
#endif