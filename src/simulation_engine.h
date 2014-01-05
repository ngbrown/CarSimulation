
#pragma once

#ifdef	__cplusplus
extern "C" {
#endif

#include "robot_physicalParams.h"



	 
typedef struct{
    float gps_x;
    float gps_y;
    int gpsLastUpdateTime;
    int gpsUpdateFreq;
    float gpsPermOffset_x;
    float gpsPermOffset_y;
    float gpsMaxRandomError;
    //float gpsAccumulatedRandomError_x;
    //float gpsAccumulatedRandomError_y;
    
    float compassAngle; //north is 0 degrees, 
    float compassMaxRandomError;
    float compassConstantError;
    float compassAccumulatedRandomError;
    
    int EncoderTicks_Left;
    int EncoderTicks_Right;
    float EncoderDistance_Left;
    float EncoderDistance_Right;
    
    float EncoderSlipPercentFullTurn;
    float EncoderSlipPercentFullPower;
    
}s_carSimulatedSensorReadings, *sp_carSimulatedSensorReadings;




typedef struct{
    sp_carPhysicalParameters physicalParams;
    sp_carSimulatedSensorReadings sensorSim;
    int timeStep;
    float x;
    float y;
    float angle; //north is 0 degrees, 
    float steeringAngle;
    float speed;
    float acceleration;
    float steeringPercentAngleCmd;
    float powerPercentCmd;
    int milisecondsOfLastUpdate;
    float debug;
}s_carSimulation, *sp_carSimulation;





//////////////  FUNCTION DELCARATION  //////////////
sp_carSimulatedSensorReadings initializeCarSensorSim();
sp_carSimulation initializeCarSim();
void simulation_updateSensors(sp_carSimulation sp_carSimu);
void simulation_update(sp_carSimulation sp_carSimu);
void simulation_print_data(sp_carSimulation x, FILE *file,int line);





#ifdef	__cplusplus
}
#endif
