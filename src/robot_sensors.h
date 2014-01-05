#pragma once

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct{
    float compass; //north is 0 radians, 
    int encoder_left;
    int encoder_right;
    float gps_x;
    float gps_y;
    int gps_new;
    float sonar_front;
}s_robot_sensors, *sp_robot_sensors;



typedef struct{
    float delta_forward;
    float delta_right;
    float delta_angle;
}s_encoder_decode_return;



typedef struct{
    float x;
    float y;
    float angle;
}s_fusion_return;

extern sp_carSimulation sp_simulation;

//////////////  FUNCTION DELCARATION  //////////////
sp_robot_sensors UpdateSensors();
s_encoder_decode_return encoder_decode(int newLeftCount, int newRightCount, sp_carPhysicalParameters sp_phys);
s_fusion_return sensorFusionAndMapping();






#ifdef	__cplusplus
}
#endif