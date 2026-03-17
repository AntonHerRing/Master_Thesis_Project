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

/******************* Defines *******************/

// static variables for the pendulum
#define g 9.81  //m/s^2
#define l 0.235 //m --> Pendulum Arm
#define r 0.14 //m  --> Rotor Arm


// From STM example
#define PRIMARY_PROPORTIONAL_MODE_1 300
#define PRIMARY_INTEGRAL_MODE_1     0.0
#define PRIMARY_DERIVATIVE_MODE_1   30

/************ Structs and Variables ************/

struct PID {
    // PID Parameters
    float kp;
    float kd;
    float ki;

    // Control variables
    float integral;
    float error;
    float prev_error;
};

/****************** Func Inits ******************/

bool oppositeSigns(int x, int y);
void init_pid(struct PID *PID1);
void PID_controller(struct PID *Pid_pend);