
#ifdef	__cplusplus
extern "C" {
#endif

#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdlib.h>

#include "robot_physicalParams.h"
#include "robot_helperFunctions.h"
#include "simulation_engine.h"



sp_carSimulatedSensorReadings initializeCarSensorSim()
{
    sp_carSimulatedSensorReadings x = (sp_carSimulatedSensorReadings)malloc(sizeof(s_carSimulatedSensorReadings));
    x->gps_x = 0;
    x->gps_y = 0;
    x->gpsLastUpdateTime = 0;
    x->gpsUpdateFreq = 10;
    x->gpsPermOffset_x = 0;//-.034;
    x->gpsPermOffset_y = 0;//.167;
    x->gpsMaxRandomError = 1.867;
    
    x->compassAngle = 0;
    x->compassMaxRandomError = deg2rad(5.4345);
    x->compassConstantError = deg2rad(-0.657);
    x->compassAccumulatedRandomError = 0;
    
    x->EncoderTicks_Left = 0;
    x->EncoderTicks_Right = 0;
    x->EncoderDistance_Left = 0;
    x->EncoderDistance_Right = 0;
    x->EncoderSlipPercentFullTurn = 5.6542;
    x->EncoderSlipPercentFullPower = 9.324;
    return x;
}



sp_carSimulation initializeCarSim(){
    
    srand(time(NULL));
    sp_carSimulation x = (sp_carSimulation)malloc(sizeof(s_carSimulation));
    x->physicalParams = initializeCarPhy();
    x->sensorSim = initializeCarSensorSim();
    x->timeStep = 5;
    x->x = 0;
    x->y = 0;
    x->angle = 0;
    x->steeringAngle = deg2rad(0);
    x->speed = 0;
    x->acceleration = 0;
    x->steeringPercentAngleCmd = 0;
    x->powerPercentCmd = 0;
    x->milisecondsOfLastUpdate = 0;
    x->debug = 0;
    return x;
}






void simulation_updateSensors(sp_carSimulation sp_carSimu)
{
    sp_carSimulatedSensorReadings sen = sp_carSimu->sensorSim;
    
    //update GPS with fixed and random offsets
    if(sen->gpsLastUpdateTime+1000/sen->gpsUpdateFreq <= sp_carSimu->milisecondsOfLastUpdate)
    {
        sen->gpsLastUpdateTime = sp_carSimu->milisecondsOfLastUpdate;
        float randomAngle = local_PI_over_2*((rand() % 100)/100.0);
        float randomRadius = getTriangleDist(0,sen->gpsMaxRandomError);
        //float randomRadius = sen->gpsMaxRandomError * ((rand() % 100)/100.0);
        sen->gps_x = sp_carSimu->x + sen->gpsPermOffset_x + randomRadius * cos(randomAngle);
        sen->gps_y = sp_carSimu->y + sen->gpsPermOffset_y + randomRadius * sin(randomAngle);
    }
    
    //update encoders
    float noSlipDist = (sp_carSimu->speed * sp_carSimu->timeStep)/1000;
    float slipPercent = sen->EncoderSlipPercentFullPower*fAbs(sp_carSimu->powerPercentCmd/100.0);
    slipPercent += sen->EncoderSlipPercentFullTurn*fAbs(sp_carSimu->steeringAngle/sp_carSimu->physicalParams->maxSteeringAngle);
    slipPercent = fLimit(slipPercent,0,100);
    float slipDist = noSlipDist*(100-slipPercent)/100.0;
    //actual distance of each encoder is:
    
    if(fAbs(sp_carSimu->steeringAngle) < .0005)
    {
        sen->EncoderDistance_Left += slipDist;
        sen->EncoderDistance_Right += slipDist;
    }else{
        float circleRadius = fAbs(sp_carSimu->physicalParams->wheelBase/tan(sp_carSimu->steeringAngle));
        circleRadius += sp_carSimu->physicalParams->wheelS2sWidth/2;
        float angleTraversed = slipDist/circleRadius;
#define EWs2s sp_carSimu->physicalParams->encoderWidth
        if(sp_carSimu->steeringAngle>0)
        {
            sen->EncoderDistance_Left += (circleRadius - EWs2s/2) * sin(angleTraversed);
            sen->EncoderDistance_Right += (circleRadius + EWs2s/2) * sin(angleTraversed);
        }else{
            sen->EncoderDistance_Left += (circleRadius + EWs2s/2) * sin(angleTraversed);
            sen->EncoderDistance_Right += (circleRadius - EWs2s/2) * sin(angleTraversed);
        }
        
    }
    
    float tickDist = local_PI_x_2 * sp_carSimu->physicalParams->encoderWheelRadius/sp_carSimu->physicalParams->encoderTicksPerRev;
    int ticks = (int)(sen->EncoderDistance_Left/tickDist);
    sen->EncoderTicks_Left += ticks;
    sen->EncoderDistance_Left -= ticks*tickDist;
    
    ticks = (int)(sen->EncoderDistance_Right/tickDist);
    sen->EncoderTicks_Right += ticks;
    sen->EncoderDistance_Right -= ticks*tickDist;
    
    
    //update compass
#define MAX_ERROR sen->compassMaxRandomError
    sen->compassAccumulatedRandomError = getTriangleDist(sp_carSimu->angle,MAX_ERROR);
    sen->compassAngle = sen->compassAccumulatedRandomError + sen->compassConstantError;
}




void simulation_update(sp_carSimulation sp_carSimu)
{
#define TIME_STEP sp_carSimu->timeStep

    int newTime = sp_carSimu->milisecondsOfLastUpdate + TIME_STEP;
    //new desired speed is calculated
    //current speed is translated into motor power, which is acceleration ability
    //acceleration goes to velocity, then you can calculate the new speed
    
    float power = sp_carSimu->physicalParams->maxMotorPower * sp_carSimu->powerPercentCmd/100;
    float drag_per_speed = sp_carSimu->physicalParams->maxMotorPower/sp_carSimu->physicalParams->maxVelocity;
    power -= drag_per_speed*sp_carSimu->speed;
    
    //check if energy will be depleted
    float newEnergy = power*TIME_STEP/1000.;
    float oldEnergy = .5*sp_carSimu->physicalParams->mass * fAbs(sp_carSimu->speed)*sp_carSimu->speed;
    float totalEnergy = newEnergy + oldEnergy;
    if(fAbs(totalEnergy) < fAbs(newEnergy))
    {
        power = 0;
        newEnergy = 0;
        totalEnergy= 0;
        sp_carSimu->speed = 0;
    }

    
    //tuo = f*r, f=tuo/r, P=tuo*w=f*r*w, v=w*r, P=f*v
    float force = 0;
    if(power != 0)
    {
        force = power / fAbs(sp_carSimu->speed);
    }
    force = fLimit(force, -1*sp_carSimu->physicalParams->stallForce,sp_carSimu->physicalParams->stallForce);
    float newAcc = force/sp_carSimu->physicalParams->mass;
    

    sp_carSimu->acceleration = newAcc;
    sp_carSimu->speed += (sp_carSimu->acceleration*TIME_STEP)/1000;
    
    
    
    float stearingAngleCommand = sp_carSimu->steeringPercentAngleCmd/100.0 * sp_carSimu->physicalParams->maxSteeringAngle;
    stearingAngleCommand += getTriangleDist(0,sp_carSimu->physicalParams->steeringPlay);
    //stearingAngleCommand += ((rand() % 200 - 100)/100.0) * sp_carSimu->physicalParams->steeringPlay;
    float delta = stearingAngleCommand - sp_carSimu->steeringAngle;
    float maxStearingAngleChange = (sp_carSimu->physicalParams->maxSteeringVelocity * TIME_STEP)/1000;
    if(fAbs(delta) < maxStearingAngleChange)
    {
        sp_carSimu->steeringAngle += delta;
    }else{
        sp_carSimu->steeringAngle += maxStearingAngleChange * fSign(delta);
    }
    
    float distance = (sp_carSimu->speed * TIME_STEP)/1000;
    
    if(fAbs(sp_carSimu->steeringAngle)>.0005)
    {
        float radius = fAbs(sp_carSimu->physicalParams->wheelBase/tan(sp_carSimu->steeringAngle));
        radius += sp_carSimu->physicalParams->wheelS2sWidth/2;
        
        float newAngle = sp_carSimu->angle + (distance/radius)*fSign(sp_carSimu->steeringAngle);
        //sp_carSimu->debug = newAngle;
        if(fSign(sp_carSimu->steeringAngle) == 1)
        {
            sp_carSimu->x += radius*(cos(newAngle-local_PI_over_2)-cos(sp_carSimu->angle-local_PI_over_2));
            sp_carSimu->y += radius*(sin(newAngle-local_PI_over_2)-sin(sp_carSimu->angle-local_PI_over_2));
        }else{
            sp_carSimu->x += radius*(cos(newAngle+local_PI_over_2)-cos(sp_carSimu->angle+local_PI_over_2));
            sp_carSimu->y += radius*(sin(newAngle+local_PI_over_2)-sin(sp_carSimu->angle+local_PI_over_2));
        }
        
        sp_carSimu->angle = normalizeRadianAngle(newAngle);
    }else{
        sp_carSimu->x += distance*cos(sp_carSimu->angle);
        sp_carSimu->y += distance*sin(sp_carSimu->angle);
    }
    
    sp_carSimu->milisecondsOfLastUpdate = newTime;
    
    
    simulation_updateSensors(sp_carSimu);
}


void simulation_print_data(sp_carSimulation x, FILE *file,int line)
{
    fprintf(file,"%d,%d,%f,%f,%f,",line,x->milisecondsOfLastUpdate,x->x,x->y,x->angle);
    fprintf(file,"%f,%f,%f,",x->steeringAngle,x->speed,x->acceleration);
    fprintf(file,"%f,%f,%f,",x->steeringPercentAngleCmd,x->powerPercentCmd,x->debug);
    
    //printf("%d,%d,%f,%f,%f,",line,x->milisecondsOfLastUpdate,x->x,x->y,x->angle);
    //printf("%f,%f,%f,",x->steeringAngle,x->speed,x->acceleration);
    //printf("%f,%f,%f,",x->steeringPercentAngleCmd,x->powerPercentCmd,x->debug);
    
#define sen x->sensorSim
    fprintf(file,"%f,%f,",sen->gps_x,sen->gps_y);
    fprintf(file,"%f,%d,%d,",sen->compassAngle,sen->EncoderTicks_Left,sen->EncoderTicks_Right);
    
    //printf("%f,%f,",sen->gps_x,sen->gps_y);
    //printf("%f,%d,%d,",sen->compassAngle,sen->EncoderTicks_Left,sen->EncoderTicks_Right);
}


#ifdef	__cplusplus
}
#endif