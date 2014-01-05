#pragma once

#ifdef	__cplusplus
extern "C" {
#endif




#define SIMULATION_ACTIVE
//#define TESTS





typedef struct{
	float MS[2001],x[2001],y[2001],angle[2001],steeringAngle[2001],speed[2001],ACC[2001],stearingCmd[2001],PowerCmd[2001],debug[2001];
	float GPS_x[2001],GPS_y[2001],Compass[2001],Encoder_L[2001],Encoder_R[2001],fusion_x[2001],fusion_y[2001],fusion_angle[2001];
}s_simulationReturnItem, *sp_simulationReturnItem;




//////////////  FUNCTION DELCARATION  //////////////
sp_simulationReturnItem simulation_main();


#ifdef	__cplusplus
}
#endif