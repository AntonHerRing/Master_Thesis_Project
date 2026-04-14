#include "Encoder.h"


void init_rotary_encoder(void){
    // initiate GPIOs
    gpio_init(Phase_A);
    gpio_set_dir(Phase_A, GPIO_IN);
    gpio_pull_up(Phase_A);

    gpio_init(Phase_B);
    gpio_set_dir(Phase_B, GPIO_IN);
    gpio_pull_up(Phase_B);
}

// Positive value over zero, Negative value under zero
// Cant show values higher then 180
float get_encoder_relative_angle(int local_count){
    local_count = local_count % ENCODER_SPR;
    local_count = local_count >= 0 ? local_count : local_count + ENCODER_SPR;

    float deg = (float)local_count * (360.0 / ENCODER_SPR);
    if(deg > 180)
        deg = -(180.0 + (180.0 - deg));
    
    return deg;
}

// Gets the exakt angle of the pendulum in positive value
// Snaps to 359 if it goes below zero.
float get_encoder_angle(int local_count){
    local_count = local_count % ENCODER_SPR;
    local_count = local_count >= 0 ? local_count : local_count + ENCODER_SPR;

    float deg = (float)local_count * (360.0 / ENCODER_SPR);
    
    return deg;
}
//same as function above, but gives in radians
float get_encoder_radian(int local_count){
    local_count = local_count % ENCODER_SPR;
    local_count = local_count >= 0 ? local_count : local_count + ENCODER_SPR;
    float radian = (float)local_count * (2*PI/ ENCODER_SPR);
    
    return radian;
}

// Utilizes angle scaler for continous reading of pos and neg
//Note: several turns to one side require same amount of turns
//back to reset it to between pos and neg. 
float get_encoder_angle_alt(int local_count){
    local_count = local_count % ENCODER_SPR;
    float deg = (float)local_count * ENCODER_ANGLE_SCALE;
    
    return deg;
}
//full continuity
float get_encoder_angle_continous(int local_count){
    float deg = (float)local_count * ENCODER_ANGLE_SCALE;
    
    return deg;
}

float get_encoder_steps(int local_count){
    return  local_count % ENCODER_SPR;
}
