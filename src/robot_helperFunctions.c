
#include "robot_helperFunctions.h"

#ifdef	__cplusplus
extern "C" {
#endif

#include <math.h>

int fSign(float x){
    if(x<0){
        return -1;
    }else{
        return 1;
    }
};

float fLimit(float x, float low, float high){
    if(x<low){
        return low;
    }else if(x>high){
        return high;
    }else{
        return x;
    }
};

float fAbs(float x){
    if(x < 0){
        return x * -1;
    }else{
        return x;
    }
};



float getRadianAngleDif(float start, float finish){
    float dif = finish - start;
    if(dif > local_PI){
        dif -= local_PI_x_2;
    }else if(dif < -local_PI){
        dif += local_PI_x_2;
    }
    return dif;
};


float normalizeRadianAngle(float x){
    return x + ((int)(x/(local_PI+.5)))*-local_PI_x_2;
};



float getTriangleDist(float mean, float cutoff){
    float randomPercent = (rand() % 1000)/1000.0;
    if(randomPercent < .5)
    {
        return cutoff * (sqrt(2*randomPercent) - 1) + mean;
    }else{
        return cutoff * ( 1 - sqrt(2*(1-randomPercent)) ) + mean;
    }
    
};




#ifdef	__cplusplus
}
#endif