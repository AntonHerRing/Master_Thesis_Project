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

// LET Labels and Local communication variables

label_t label_Enc;        /* Data label used for LET tasks. */
label_t label_Motor;        /* Data label used for LET tasks. */
label_t label_Contr;        /* Data label used for LET tasks. */

LetTask_t letEncTsk;    /*Handle for the LET rotary encoder task. */
LetTask_t letMotorTsk;  /*Handle for the LET stepper motor task. */
LetTask_t letContrTsk;  /*Handle for the LET Control task. */
LetTask_t letPrintTsk;  /*Handle for the LET Print task. */

uint32_t* task_Enc;      /* Pointer to the local data of label ENC by Encoder task. */
uint32_t  task_Enc_data; /* Local copy of label ENC owned by LET Encoder task. */
uint32_t* PrintTask_Enc;      /* Pointer to the local data of label Enc by Print task. */
uint32_t  PrintTask_Enc_data; /* Local copy of label Enc owned by Print LET task. */
uint32_t* ContrTask_Enc;      /* Pointer to the local data of label ENC by Control task. */
uint32_t  ContrTask_Enc_data; /* Local copy of label ENC owned by Control LET task. */

uint32_t* task_Motor;      /* Pointer to the local data of label Motor by Motor task. */
uint32_t  task_Motor_data; /* Local copy of label Motor owned by LET Motor task. */
uint32_t* PrintTask_Motor;      /* Pointer to the local data of label Motor by Print task. */
uint32_t  PrintTask_Motor_data; /* Local copy of label Motor owned by Print LET task. */

uint32_t* task_Contr;      /* Pointer to the local data of label Contr by Control task. */
uint32_t  task_Contr_data; /* Local copy of label Contr owned by LET Control task. */
uint32_t* MotorTask_Contr;      /* Pointer to the local data of label Contr by Motor task. */
uint32_t  MotorTask_Contr_data; /* Local copy of label Contr owned by Motor LET task. */

// Rotary Encoder Interrupt Variables
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

/**
 * @brief Initialization function of Control LET task.
 */
void vLetContrTask_init(void);

/**
 * @brief Job function of Control LET task.
 */
void vLetContrTask_job(void);


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

    // Initialize the Interrupts on the two A and B ports
    gpio_set_irq_enabled_with_callback(Phase_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled(Phase_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    /* Create a label and LET tasks that read/write from it. */
    xLetInitLabel("Enc", sizeof(uint32_t), &label_Enc, LET_COM_COPY);
    xLetInitLabel("Motor", sizeof(uint32_t), &label_Motor, LET_COM_COPY);
    xLetInitLabel("Contr", sizeof(uint32_t), &label_Contr, LET_COM_COPY);

    xLetTaskCreate(vLetEncTask_init, vLetEncTask_job, "LET_Enc_Task", 512, 5, 100, 100, 0, CORE0, &letEncTsk);
    xLetTaskCreate(vLetMotorTask_init, vLetMotorTask_job, "LET_Motor_Task", 512, 5, 50, 50, 0, CORE0, &letMotorTsk);
    xLetTaskCreate(vLetPrintTask_init, vLetPrintTask_job, "LET_Print_Task", 512, 5, 100, 100, 0, CORE0, &letPrintTsk);
    xLetTaskCreate(vLetContrTask_init, vLetContrTask_job, "LET_Control_Task", 512, 5, 100, 100, 0, CORE0, &letContrTsk);
    
    vTaskStartScheduler();  /* Start the scheduler. */
    
    while (true) { 
        sleep_ms(1000); /* Should not reach here... */
    }
}

/*-----------------------------------------------------------*/

void vLetEncTask_init(void) {
    task_Enc = &task_Enc_data;    /* Initialize the pointer to the local buffer for label ENC */

    xLetTaskRegisterWrite(&letEncTsk, &label_Enc, (void*) &task_Enc);    /* Register the write access for label A */    
}
/*-----------------------------------------------------------*/

void vLetEncTask_job(void) {
    /******** Init static var ********/
    static float deg = 0.0;

    /******** Main function ********/

    deg = get_encoder_angle(count);

    (*task_Enc) = (uint32_t)deg; //write any inputs
}
/*-----------------------------------------------------------*/

void vLetMotorTask_init(void) {
    task_Motor = &task_Motor_data;    /* Initialize the pointer to the local buffer for label Motor */
    MotorTask_Contr = &MotorTask_Contr_data;

    xLetTaskRegisterWrite(&letMotorTsk, &label_Motor, (void*) &task_Motor);    /* Register the write access for label Motor */   
    xLetTaskRegisterRead(&letMotorTsk, &label_Contr, (void*) &MotorTask_Contr); /* Register the read access for label Contr */ 
}
/*-----------------------------------------------------------*/

void vLetMotorTask_job(void) {
    /******** Init static var ********/
    
    static int max_pos = 25;
    static int min_pos = -25;

    //local stepper motor dir
    static int l_dir = 1;

    static float motor_deg = 0.0;
    static float relativ_deg = 0.0;
    static float offset = 0.0;
    static int first_time = 20;

    static uint32_t Contr_sig = 1;   // 1 == Go, -1 == Stop

    /******** Main function ********/
    if(first_time > 0){     //wait until the stepper motor value has stabilized
        first_time--;
        offset = get_stepper_angle();
        move_stepper_by(0.2);
        sleep_ms(10);
        move_stepper_by(-0.2);
        printf("First Time: %d\tOffset: %f\n", first_time, offset);
    }
    else{
        motor_deg = (get_stepper_angle() - offset);
        //relativ_deg = 180.0 - (180.0 - motor_deg);  //Pos = 0 - 180 half || Neg = 360 - 180 half
        relativ_deg = motor_deg;
        Contr_sig = *MotorTask_Contr;

        if((int)relativ_deg >= max_pos || Contr_sig == 2)
            l_dir = -1;
        else if((int)relativ_deg <= min_pos || Contr_sig == 3)
            l_dir = 1;

        //printf("Motor Control: %d\t Dir: %d\tRel Deg: %f\n", Contr_sig, l_dir, relativ_deg);

        //Contr task sends STOP signal via MotorTask_Contr when around 180 Deg
        if (Contr_sig == 0);    //do nothing
        else if (l_dir == 1)
            move_stepper_by(0.2);
        else if (l_dir == -1 )
            move_stepper_by(-0.2);

        (*task_Motor) = (uint32_t)abs(motor_deg); //write any inputs
    }
}
/*-----------------------------------------------------------*/

void vLetPrintTask_init(void) {
    PrintTask_Enc = &PrintTask_Enc_data;  /* Initialize the pointer to the local buffers */
    PrintTask_Motor = &PrintTask_Motor_data;

    xLetTaskRegisterRead(&letPrintTsk, &label_Enc, (void*) &PrintTask_Enc);    /* Register the read access for label Enc */    
    xLetTaskRegisterRead(&letPrintTsk, &label_Motor, (void*) &PrintTask_Motor);    /* Register the read access for label Motor */   
}
/*-----------------------------------------------------------*/

void vLetPrintTask_job(void) {

    /******** Main function ********/
    printf("Deg: %u\tMotor Deg: %u\r\n", *PrintTask_Enc, *PrintTask_Motor); //Read any inputs
}
/*-----------------------------------------------------------*/

void vLetContrTask_init(void) {
    task_Contr = &task_Contr_data;  /* Initialize the pointer to the local buffers */
    ContrTask_Enc = &ContrTask_Enc_data;

    xLetTaskRegisterRead(&letContrTsk, &label_Enc, (void*) &ContrTask_Enc);    /* Register the read access for label A */    
    xLetTaskRegisterWrite(&letContrTsk, &label_Contr, (void*) &task_Contr);    /* Register the read access for label B */   
}
/*-----------------------------------------------------------*/

void vLetContrTask_job(void) {

    /******** Main function ********/
    // test
    // Read Rotary Encoder angle, and send STOP signal to Control Variable for the Motor
    if (*ContrTask_Enc >= 170 && *ContrTask_Enc <= 190)     //STOP -- ~180
        (*task_Contr) = 0;
    else if(*ContrTask_Enc >= 80 && *ContrTask_Enc <= 100) //LEFT -- ~90
        (*task_Contr) = 2;
    else if(*ContrTask_Enc >= 250 && *ContrTask_Enc <= 280) //RIGHT -- ~270
        (*task_Contr) = 3;
    else
        (*task_Contr) = 1;                                  //GO

    //printf("Deg: %u\tMotor Deg: %u\r\n", *PrintTask_Enc, *PrintTask_Motor); //Read any inputs
}
/*-----------------------------------------------------------*/