
#pragma once

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include "robot_helperFunctions.h"





typedef enum{
    //length Meters, Centimeters, Millimeters
    Meters,
    //Centimeters,
    //Millimeters,
    
    //speed and acceleration: squared
    Meters_Per_Second,
    Meters_Per_Second_Squared,
    
    //mass: Kilograms, Grams
    Kilograms,
    //Grams,
    
    //power: watts
    Watts,
            
    //Time: Seconds, Milliseconds
    Seconds,
    Milliseconds,
    
    //Angle: Degrees, Radians
    //Degrees,
    Radians,
            
    //Angular velocity: Degrees, Radians
    //DegreesPerSecond,
    RadiansPerSecond,
    
}engineeringUnits;



typedef struct{
    float mass;
    float length;
    float width;
    float wheelBase;
    float wheelS2sWidth;
    float wheelRadius;
    float maxSteeringAngle;
    float maxSteeringVelocity;
    float steeringPlay;
    
    float maxMotorPower;
    float stallForce;
    float maxVelocity;
    
    float encoderWidth;
    float encoderWheelRadius;
    int encoderTicksPerRev;
    
}s_carPhysicalParameters, *sp_carPhysicalParameters;



//////////////  FUNCTION DELCARATION  //////////////
sp_carPhysicalParameters initializeCarPhy();



#ifdef	__cplusplus
}
#endif