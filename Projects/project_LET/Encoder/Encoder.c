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
float get_encoder_angle(int local_count){
    local_count = local_count % ENCODER_SPR;
    local_count = local_count >= 0 ? local_count : local_count + ENCODER_SPR;
    
    return 180.0 - (180.0 - (float)local_count * (360.0 / ENCODER_SPR));
}