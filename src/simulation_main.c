
#ifdef	__cplusplus
extern "C" {
#endif

	
/* TODO
 * -add ability to estimate angle from GPS locations
 * -break out the radial turn algorithm
 * -ramp speed to reduce slip
 * -follow a spline
 *    -probably calculate the spline off 3 points, and do the next spline when you reach the middle point
 *  
 * -test for compass failures, to know when to reject the data
 *    -possibly check the Z data(magnitudes or angles)
 * 

 */

#include <stdio.h>
#include <stdlib.h>
#include "simulation_main.h"

#define STRIP_DOWN




#include "robot_physicalParams.h"

#include "simulation_engine.h"
sp_carSimulation sp_simulation;

#include "robot_routePlanner.h"




#include "robot_sensors.h"

#ifdef TESTS
#include "Tests.h"
#endif



sp_simulationReturnItem simulation_main(){
    #ifdef TESTS
    testGetTriangleDist();
    return;
    #endif

    sp_simulation = initializeCarSim();
    FILE *file;
    file = fopen("file.csv","w"); 

    int line = 0;
    fprintf(file,"line,MS,x,y,angle,steeringAngle,speed,ACC,stearingCmd,PowerCmd,debug,");
    fprintf(file,"GPS_x,GPS_y,Compass,Encoder_L,Encoder_R,fusion_x,fusion_y,fusion_angle\n");
    simulation_print_data(sp_simulation,file, line++);
    fprintf(file,"0,0,0\n");
    int x;
    
    int goalCounter = 0;
    //float x_goal[10] = {0, 30,-30,30,-30,30,-30,30}; //criss cross
    //float y_goal[10] = {30,30, 10,-10, -30,-10 ,-20,-30}; //criss cross
    
    float x_goal[10] = {30, 30,-30,-30, 20,20,-20,-20}; //turn left box
    float y_goal[10] = {-20 ,30,  30,-30,-30,20 ,20,-20}; //turn left box
    s_routePlanner_return cmd = gotoWaypoint(0, 0, x_goal[0], y_goal[0], 0);
    sp_simulation->steeringPercentAngleCmd = cmd.steeringCmd;
    sp_simulation->powerPercentCmd = cmd.power;
    
	sp_simulationReturnItem sp_returnData = (sp_simulationReturnItem)malloc(sizeof(s_simulationReturnItem));

    for(x = 0; x<20000; x++)
    {

        simulation_update(sp_simulation);

        s_fusion_return fusion = sensorFusionAndMapping();
        s_routePlanner_return cmd = gotoWaypoint(fusion.x, fusion.y, x_goal[goalCounter], y_goal[goalCounter], fusion.angle);
        sp_simulation->steeringPercentAngleCmd = cmd.steeringCmd;
        sp_simulation->powerPercentCmd = cmd.power;
        if(cmd.finished)
        {
            goalCounter++;
        }

        if(x%10==0)
        {
			sp_returnData->x[x/10] = sp_simulation->x; sp_returnData->y[x/10] = sp_simulation->y; sp_returnData->angle[x/10] = sp_simulation->angle;
			
            simulation_print_data(sp_simulation, file, line++);
            fprintf(file,"%f,%f,%f\n",fusion.x,fusion.y,fusion.angle);
			sp_returnData->fusion_x[x/10] = fusion.x; sp_returnData->fusion_y[x/10] = fusion.y; sp_returnData->fusion_angle[x/10] = fusion.angle;
            //printf("%f,%f,%f\n",fusion.x,fusion.y,fusion.angle);
        }
    }
    fclose(file); /*done!*/ 
	return sp_returnData;
}


#ifdef	__cplusplus
}
#endif