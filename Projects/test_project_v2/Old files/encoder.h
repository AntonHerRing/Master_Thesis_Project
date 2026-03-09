#ifndef ENCODER_H
#define ENCODER_H

void init_encoder();
void deinit_encoder();
int readPendulumPos();      // Read Pendulum Position/Steps, range: [-600, 600]
float readPendulumAngle();  // Read Pendulum Angle, range: [0, 360)
void encoder_task();

#endif

/*
**********     README     **********
TO WRITE CODE
    - put RED.c, RED.h, encoder.h files under the same folder as where this file is located
    - include header file: #include "encoder.h"
    - always call 'encoder_init()' in 'main' function
    
TO BUILD
    gcc -Wall -pthread -o EXECUTABLE [PROGRAM].c encoder.c RED.c -lpigpiod_if2 -lm
    
TO RUN
    sudo pigpiod # if the daemon is not already running
    sudo ./EXECUTABLE
*/