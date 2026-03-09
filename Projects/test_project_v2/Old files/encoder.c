/* 
    An interface modified from test_RED.c.
    This is an improved verison to read from rotary encoder using software, step missing problem has been largely mitigated by the usage of pigpiod_if2 library.
    The pigpio library uses the DMA circuits to do stuff outside the CPU so it isn't affected by CPU multi-tasking. May only possible to run on CORE1. 
    Reference from website link http://abyz.me.uk/rpi/pigpio/examples.html#pigpiod_if2%20code
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <pigpiod_if2.h>

#include "RED.h"

//#include "registers.h"
//#include "logging.h"

#define PULSE_PER_ROUND 600
#define DEGREE_PER_ROUND 360

int pi;
RED_t *renc;

int optGpioA = 27;
int optGpioB = 23;
int optGlitch = 10;
int optSeconds = 0;
int optMode = RED_MODE_DETENT;
char *optHost   = NULL;
char *optPort   = NULL;

void cbf(int value)
{
   // printf("%d\n", value);
}

void init_encoder(){

    renc = RED(pi, optGpioA, optGpioB, optMode, cbf);
    RED_set_glitch_filter(renc, optGlitch);
}

void deinit_encoder(){
    RED_cancel(renc);
}

// niceModulo(370, 360) = 10
// niceModulo(-370, 360) = 350
float niceModulo(float i, float mod){
    if(i >= 0){
        return fmod(i, mod);
    }
    else{
        return fmod(i, mod) + mod;
    }
}

int readPendulumPos(){
    int pos = RED_get_position(renc);
    if (pos >= PULSE_PER_ROUND) RED_set_position(renc, pos-PULSE_PER_ROUND);
    if (pos <= -PULSE_PER_ROUND) RED_set_position(renc, pos+PULSE_PER_ROUND);
    return pos;
}

// Rescales pos to give the actual pendulum angle in degrees. 
// Use fabs and modulo 360 to ignore direction and number of rotations. 
float readPendulumAngle(){
    int pos = RED_get_position(renc);
    float accumulatedAngle = (float) pos/ PULSE_PER_ROUND * DEGREE_PER_ROUND;
	
	return niceModulo(accumulatedAngle, DEGREE_PER_ROUND);
}

void encoder_task(){
    float angle = readPendulumAngle();
    int pos = readPendulumPos();
    //printf("Pendulum Angle: %.2f\n", angle);
    write_float_register(angle, &pend_angle_register);
    write_float_register(pos, &pend_step_register);
}
