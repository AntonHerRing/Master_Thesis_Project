#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "bsp.h"
#include "let.h"

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


//function definition
extern void L6474_StepClockHandler(uint8_t deviceId);

label_t label_A;        /* Data label used for LET tasks. */
label_t label_B;        /* Data label used for LET tasks. */

LetTask_t letEncTsk;    /*Handle for the LET rotary encoder task. */
LetTask_t letMotorTsk;  /*Handle for the LET stepper motor task. */
LetTask_t letPrintTsk;  /*Handle for the LET Print task. */

uint32_t* task1_A;      /* Pointer to the local data of label A by task 1. */
uint32_t  task1_A_data; /* Local copy of label A owned by LET task 1. */
uint32_t* PrintTask_A;      /* Pointer to the local data of label A by Print task. */
uint32_t  PrintTask_A_data; /* Local copy of label A owned by Print LET task. */

uint32_t* task2_B;      /* Pointer to the local data of label B by task 2. */
uint32_t  task2_B_data; /* Local copy of label B owned by LET task 2. */
uint32_t* PrintTask_B;      /* Pointer to the local data of label B by Print task. */
uint32_t  PrintTask_B_data; /* Local copy of label B owned by Print LET task. */

//The Rotary (Gray code) Pulses
volatile bool Pulse_A = false;
volatile bool Pulse_B = false;

volatile int32_t count = 0;
volatile int dir = 0;

/**
 * @brief Initialization function of Rotary Encoder LET task.
 */
void vLetEncTask_init(void);

/**
 * @brief Job function of Rotary Encoder LET task.
 */
void vLetEncTask_job(void);

/**
 * @brief Initialization function of Motor LET task.
 */
void vLetMotorTask_init(void);

/**
 * @brief Job function of Motor LET task.
 */
void vLetMotorTask_job(void);

/**
 * @brief Initialization function of Print LET task.
 */
void vLetPrintTask_init(void);

/**
 * @brief Job function of Print LET task.
 */
void vLetPrintTask_job(void);


/*************************************************************/

/**
 * @brief Interrupt handler callback function.
 */
void gpio_callback(uint gpio, uint32_t events) {
    static int buffer = 0x00;
    if (events & (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL)) {
        // Rising or falling edge detected
        if (gpio == Phase_A){ 
            dir = gpio_get(Phase_A) == gpio_get(Phase_B) ? 1 : -1;
            count += dir >= 0 ? 1 : -1;
        }
        if (gpio == Phase_B){
            dir = gpio_get(Phase_A) != gpio_get(Phase_B) ? 1 : -1;
            count += dir >= 0 ? 1 : -1;
        }
    }
}


/*************************************************************/

/**
 * @brief Main function.
 * 
 * @return int 
 */
int main()
{
    BSP_Init();             /* Initialize all components on the lab-kit. */
    init_rotary_encoder();  /* Initialize the Rotary Encoder. */
    init_motor();           /* Initialize the Stepper Motor*/
    
    if (xLetInit() == pdFALSE) {                    /* Initialize the LET module. */
        while (true);
    }

    gpio_set_irq_enabled_with_callback(Phase_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled(Phase_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    /* Create a label and LET tasks that read/write from it. */
    xLetInitLabel("A", sizeof(uint32_t), &label_A, LET_COM_COPY);
    xLetInitLabel("B", sizeof(uint32_t), &label_B, LET_COM_COPY);

    xLetTaskCreate(vLetEncTask_init, vLetEncTask_job, "LET_Enc_Task", 512, 5, 100, 100, 0, CORE0, &letEncTsk);
    xLetTaskCreate(vLetMotorTask_init, vLetMotorTask_job, "LET_Motor_Task", 512, 5, 50, 50, 0, CORE0, &letMotorTsk);
    xLetTaskCreate(vLetPrintTask_init, vLetPrintTask_job, "LET_Print_Task", 512, 5, 100, 100, 0, CORE0, &letPrintTsk);
    
    vTaskStartScheduler();  /* Start the scheduler. */
    
    while (true) { 
        sleep_ms(1000); /* Should not reach here... */
    }
}

/*-----------------------------------------------------------*/

void vLetEncTask_init(void) {
    task1_A = &task1_A_data;    /* Initialize the pointer to the local buffer for label A */

    xLetTaskRegisterWrite(&letEncTsk, &label_A, (void*) &task1_A);    /* Register the write access for label A */    
}
/*-----------------------------------------------------------*/

void vLetEncTask_job(void) {
    /******** Init static var ********/
    static float deg = 0.0;

    /******** Main function ********/

    deg = get_encoder_angle(count);

    (*task1_A) = (uint32_t)deg; //write any inputs
}
/*-----------------------------------------------------------*/

void vLetMotorTask_init(void) {
    task2_B = &task2_B_data;    /* Initialize the pointer to the local buffer for label A */

    xLetTaskRegisterWrite(&letMotorTsk, &label_B, (void*) &task2_B);    /* Register the read access for label A */    
}
/*-----------------------------------------------------------*/

void vLetMotorTask_job(void) {
    /******** Init static var ********/
    
    static int max_pos = 50;
    static int min_pos = 0;

    static int dir = 1;

    static float motor_deg = 0.0;

    /******** Main function ********/

    motor_deg = get_stepper_angle();

    if(abs((int)(motor_deg)) >= max_pos)
        dir = -1;
    else if(abs((int)(motor_deg)) <= min_pos)
        dir = 1;

    if (dir == 1)
        move_stepper_by(0.2);
    else if (dir == -1)
        move_stepper_by(-0.2);

    (*task2_B) = (uint32_t)abs(motor_deg); //write any inputs

    //printf("Motor deg: %d\tTest: %d\n", (uint32_t)abs(motor_deg), *task2_B);

    //printf("%u\r\n", *task2_B); //Read any inputs
}
/*-----------------------------------------------------------*/

void vLetPrintTask_init(void) {
    PrintTask_A = &PrintTask_A_data;  /* Initialize the pointer to the local buffers */
    PrintTask_B = &PrintTask_B_data;

    xLetTaskRegisterRead(&letPrintTsk, &label_A, (void*) &PrintTask_A);    /* Register the read access for label A */    
    xLetTaskRegisterRead(&letPrintTsk, &label_B, (void*) &PrintTask_B);    /* Register the read access for label B */   
}
/*-----------------------------------------------------------*/

void vLetPrintTask_job(void) {

    /******** Main function ********/
    printf("Deg: %u\tMotor Deg: %u\r\n", *PrintTask_A, *PrintTask_B); //Read any inputs
    //printf("Motor Deg: %u\r\n", *PrintTask_B);
}
/*-----------------------------------------------------------*/