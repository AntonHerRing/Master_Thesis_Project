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
//#include "hardware/pio.h"
//#include "hardware/"

#include "Drivers/L16474/motor_rpi3b_interface.h"
#include "Drivers/L16474/l6474.h"
#include "Drivers/L16474/steppermotor.h"

#include "Control/Control.h"
#include "Encoder/Encoder.h"

/*
GPIO9::     CS
GPIO10::    SCK
GPIO11::    MOSI
GPIO12::    MISO
*/
#define SPI_CS 9
#define SPI_SCK 10
#define SPI_MOSI 11
#define SPI_MISO 12

//encoder gpio ports
#define Phase_A 40
#define Phase_B 39


//TaskHandle_t    blinkTsk; /* Handle for the LED task. */
//TaskHandle_t    acclTsk; /* Handle for the accelerometer task. */
TaskHandle_t    encTsk; /* Handle for the rotary encoder task. */
TaskHandle_t    motorTsk; /* Handle for the stepper motor task. */

/**
 * @brief Blink task.
 * 
 * @param args Task period (uint32_t).
 */
//void blink_task(void *args);
void enc_task(void *args);
void motor_task(void *args);

//function definition
int grayTo_int(bool Enc_A, bool Enc_B);

void init_rotary_encoder(void);

extern void L6474_StepClockHandler(uint8_t deviceId);

//The Rotary (Gray code) Pulses
volatile bool Pulse_A = false;
volatile bool Pulse_B = false;

volatile int32_t count = 0;
volatile int dir = 0;

//volatile bool A_first = false;
//volatile bool B_first = false;


/*************************************************************/

/**
 * @brief Interrupt handler callback function.
 */
void gpio_callback(uint gpio, uint32_t events) {
    static bool A_first = false;
    static bool B_first = false;

    static int buffer = 0x00;
    if (events & (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)) {
        // Rising or falling edge detected
        if (gpio == Phase_A){ 
            dir = gpio_get(Phase_A) == gpio_get(Phase_B) ? 1 : -1;
            count += dir >= 0 ? 1 : -1;
            //count++;
        }
        if (gpio == Phase_B){
            dir = gpio_get(Phase_A) != gpio_get(Phase_B) ? 1 : -1;
            count += dir >= 0 ? 1 : -1;
            //count++;
        }
    }
}


/**
 * @brief Main function.
 * 
 * @return int 
 */
int main()
{
    BSP_Init();             /* Initialize all components on the lab-kit. */
    init_rotary_encoder();  /* Initialize the Rotary Encoder. */

    init_motor();

    //gpio_set_irq_enabled_with_callback(Phase_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    //gpio_set_irq_enabled(Phase_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    /* Create the tasks. */
    //xTaskCreate(enc_task, "Enc task", 512, (void*) 2, 2, &encTsk);
    xTaskCreate(motor_task, "Motor task", 512, (void*) 1, 2, &motorTsk);

    vTaskStartScheduler();  /* Start the scheduler. */
    
    while (true) { 
        sleep_ms(1000); /* Should not reach here... */
    }
}
/*-----------------------------------------------------------*/


void enc_task(void *args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   // Get period (in ticks) from argument.

    float deg = 0.0;

    for (;;) {

        deg = get_encoder_angle(count);
        
        //printf("Deg: %f\n", deg);
        
        //last step in loop
        vTaskDelayUntil(&xLastWakeTime, xPeriod);   // Wait for the next release. 
    }   
}

void motor_task(void *args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   // Get period (in ticks) from argument.

    int max_pos = 50;
    int min_pos = 0;

    int dir = 1;

    float motor_deg = 0.0;
    float deg_offset = 0.0;

    bool first_time = true;

    printf("Calibrating Motor Position...\n");
    sleep_ms(10);
    move_stepper_by(1.0);
    sleep_ms(10);
    move_stepper_by(-1.0);
    int32_t test1 = get_stepper_angle();
    L6474_SetHome(0, (int32_t)(get_stepper_angle()*MOTOR_STEPS_PER_DEGREE));
    sleep_ms(20);
    int32_t test2 = get_stepper_angle();

    //****Calibration Stop **** *

    uint8_t btn1 = 0;
    uint8_t btn2 = 0;
    uint8_t btn3 = 0;
    uint8_t btn4 = 0;

    uint8_t buttons = 0;
    int dummy = 0;
    static bool Off = false;
    float encoder = 0;

    static float collector = 0; 

    float curr_pos = 0;
    float pre_pos = 0;

    for (;;) {
        /*motor_deg = get_stepper_angle();
        printf("Motor deg: %d\n", abs((int)(motor_deg)));

        if(abs((int)(motor_deg)) >= max_pos)
            dir = -1;
        else if(abs((int)(motor_deg)) <= min_pos)
            dir = 1;

        if (dir == 1)
            move_stepper_by(0.2);
        else if (dir == -1)
            move_stepper_by(-0.2);*/

        btn1 = BSP_GetInput(SW_5);
        btn2 = BSP_GetInput(SW_6);
        btn3 = BSP_GetInput(SW_7);
        btn4 = BSP_GetInput(SW_8);
        encoder = get_encoder_angle_continous(count);
        //printf("collector: %d\n", collector); 
        curr_pos = (float)L6474_ConvertPosition(L6474_CmdGetParam(0,L6474_ABS_POS))/MOTOR_STEPS_PER_DEGREE;
        printf("Current pos: %f\tPend Ang:%f\n", curr_pos, encoder);
        if (abs(pre_pos - curr_pos) > 100){
            printf("Anomaly Detected!");
        }

        buttons = 0x0 | (!btn1 | (!btn2 << 1) | (!btn3 << 2) | (!btn4 << 3));
        switch(buttons){
            case 1:
                collector += 0.5;
            break;
            case 2:
                collector -= 0.5;
            break;
            default:
            break;
        }
        move_stepper_to(collector);

        pre_pos = curr_pos;
     
        vTaskDelayUntil(&xLastWakeTime, xPeriod);   // Wait for the next release. 
    }   
}


