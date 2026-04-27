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

float get_encoder_angle(int local_count){
    local_count = local_count % ENCODER_SPR;
    local_count = local_count >= 0 ? local_count : local_count + ENCODER_SPR;
    
    return (float)local_count * (360.0 / ENCODER_SPR);
}

float get_encoder_angle_continous(int local_count){
    float deg = (float)local_count * ENCODER_ANGLE_SCALE;
    
    return deg;
}