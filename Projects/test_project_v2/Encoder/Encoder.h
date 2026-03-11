#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "bsp.h"

#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/structs/io_bank0.h"

#include "Drivers/L16474/motor_rpi3b_interface.h"
#include "Drivers/L16474/l6474.h"
#include "Drivers/L16474/steppermotor.h"

/****************** Defines ******************/

//Phase A and B GPIO ports for the Rotary Encoder
#define Phase_A 40
#define Phase_B 39

//Encoder Steps Per revolution
#define ENCODER_SPR 2400

/****************** Func Inits ******************/


void init_rotary_encoder(void);
float get_encoder_angle(int local_count);