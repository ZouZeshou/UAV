#include "ramp.h"

ramp_t ramp_x = RAMP_DEFAULT_INIT;
ramp_t ramp_y = RAMP_DEFAULT_INIT;

/**
 * @brief set a buff parameter for chassismotor
 * @param None
 * @return None
 * @attention None
 */
void ramp_init(ramp_t *ramp){
	ramp->count = 40;
	ramp->out = 0;
}

float ramp_cal(ramp_t *ramp){
  if(ramp->scale <=0) return 0;
  if (ramp->count++ >= ramp->scale)
    ramp->count = ramp->scale;
  
  ramp->out = ramp->count / ((float)ramp->scale);
  return ramp->out;	
}
