#ifdef	__cplusplus
extern "C" {
#endif

#include "robot_physicalParams.h"

sp_carPhysicalParameters initializeCarPhy(){
    sp_carPhysicalParameters x = (sp_carPhysicalParameters)malloc(sizeof(s_carPhysicalParameters));
    x->mass = 4.217;
    x->length = .251;
    x->width = .15;
    x->wheelBase = .2185;
    x->wheelS2sWidth = .14;
    x->wheelRadius = .025;
    x->maxSteeringAngle = deg2rad(30); //radians
    x->maxSteeringVelocity = deg2rad(50); //radians per second
    x->steeringPlay = deg2rad(5.42);//2.6478);
    x->maxMotorPower = 50; //watts
    x->stallForce = 51.32; //newton
    x->maxVelocity = 3.1; //meters per second
    x->encoderWidth = .14;
    x->encoderWheelRadius = .02;
    x->encoderTicksPerRev = 50;
    return x;
}




	

#ifdef	__cplusplus
}
#endif