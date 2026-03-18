#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
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


//Task Periods
#define T_Enc   100
#define T_Motor 50
#define T_Contr 100
#define T_Print 100

/*
This configuration was unstable,
and the Stepper motor would jump
large distances unprompted

#define T_Enc   50
#define T_Motor 50
#define T_Contr 50
#define T_Print 100
*/

/***** STM Var******/
float *current_error_steps, *current_error_rotor_steps;
float encoder_angle_slope_corr_steps;
float pendulum_position_command_steps;
float rotor_control_target_steps;
int rotor_position_steps;
float rotor_position_command_steps;
float feedforward_gain;
float encoder_position;


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

int32_t* task_Enc;      /* Pointer to the local data of label ENC by Encoder task. */
int32_t  task_Enc_data; /* Local copy of label ENC owned by LET Encoder task. */
int32_t* PrintTask_Enc;      /* Pointer to the local data of label Enc by Print task. */
int32_t  PrintTask_Enc_data; /* Local copy of label Enc owned by Print LET task. */
int32_t* ContrTask_Enc;      /* Pointer to the local data of label ENC by Control task. */
int32_t  ContrTask_Enc_data; /* Local copy of label ENC owned by Control LET task. */

int32_t* task_Motor;      /* Pointer to the local data of label Motor by Motor task. */
int32_t  task_Motor_data; /* Local copy of label Motor owned by LET Motor task. */
int32_t* PrintTask_Motor;      /* Pointer to the local data of label Motor by Print task. */
int32_t  PrintTask_Motor_data; /* Local copy of label Motor owned by Print LET task. */

int32_t* task_Contr;      /* Pointer to the local data of label Contr by Control task. */
int32_t  task_Contr_data; /* Local copy of label Contr owned by LET Control task. */
int32_t* MotorTask_Contr;      /* Pointer to the local data of label Contr by Motor task. */
int32_t  MotorTask_Contr_data; /* Local copy of label Contr owned by Motor LET task. */

// Rotary Encoder Interrupt Variables
volatile int32_t count = 0;
volatile int dir = 0;

// PID variables
struct PID Pid1;
struct PID *PID_Pend = &Pid1;

struct PID Pid2;
struct PID *PID_Rotor = &Pid2;

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

    init_pid(PID_Pend, PID_Rotor);         /* Initialize PID variables with initial values*/
    
    if (xLetInit() == pdFALSE) {                    /* Initialize the LET module. */
        while (true);
    }

    // Initialize the Interrupts on the two A and B ports
    gpio_set_irq_enabled_with_callback(Phase_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled(Phase_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    /* Create a label and LET tasks that read/write from it. */
    xLetInitLabel("Enc", sizeof(int32_t), &label_Enc, LET_COM_COPY);
    xLetInitLabel("Motor", sizeof(int32_t), &label_Motor, LET_COM_COPY);
    xLetInitLabel("Contr", sizeof(int32_t), &label_Contr, LET_COM_COPY);

    xLetTaskCreate(vLetEncTask_init, vLetEncTask_job, "LET_Enc_Task", 512, 2, T_Enc, T_Enc, 0, CORE0, &letEncTsk);
    xLetTaskCreate(vLetContrTask_init, vLetContrTask_job, "LET_Control_Task", 512, 3, T_Contr, T_Contr, 0, CORE0, &letContrTsk);
    xLetTaskCreate(vLetMotorTask_init, vLetMotorTask_job, "LET_Motor_Task", 512, 4, T_Motor, T_Motor, 0, CORE0, &letMotorTsk);
    xLetTaskCreate(vLetPrintTask_init, vLetPrintTask_job, "LET_Print_Task", 512, 5, T_Print, T_Print, 0, CORE0, &letPrintTsk);
    
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

    //deg = get_encoder_angle(count);

    (*task_Enc) = get_encoder_steps(count);

    //printf("Test ang: %d\n", (int)get_encoder_angle_alt(count));

    //(*task_Enc) = (int32_t)deg; //write any inputs
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

    static int32_t Contr_sig = 1;   // 1 == Go, -1 == Stop

    /******** Main function ********/
    if(first_time > 0){     //wait until the stepper motor value has stabilized
        first_time--;
        offset = get_stepper_angle();
        move_stepper_by(0.2);
        sleep_ms(10);
        move_stepper_by(-0.2);
        printf("First Time: %d\tOffset: %f\n", first_time, offset);
        sleep_ms(10);
    }
    /*else{
        motor_deg = (get_stepper_angle() - offset);
        Contr_sig = *MotorTask_Contr;

        if((int)motor_deg >= max_pos || Contr_sig == 2)
            l_dir = -1;
        else if((int)motor_deg <= min_pos || Contr_sig == 3)
            l_dir = 1;

        //Contr task sends STOP signal via MotorTask_Contr when around 180 Deg
        if (Contr_sig == 0);    //do nothing
        else if (l_dir == 1)
            move_stepper_by(0.2);
        else if (l_dir == -1 )
            move_stepper_by(-0.2);

        (*task_Motor) = (int32_t)motor_deg; //write any inputs
    }*/
    else{
        L6474_GoTo(0, *MotorTask_Contr);
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
    printf("Deg: %d\tMotor Deg: %d\r\n", get_encoder_angle(*PrintTask_Enc), *PrintTask_Motor); //Read any inputs
}
/*-----------------------------------------------------------*/

void vLetContrTask_init(void) {
    task_Contr = &task_Contr_data;  /* Initialize the pointer to the local buffers */
    ContrTask_Enc = &ContrTask_Enc_data;

    xLetTaskRegisterRead(&letContrTsk, &label_Enc, (void*) &ContrTask_Enc);    /* Register the read access for label A */    
    xLetTaskRegisterWrite(&letContrTsk, &label_Contr, (void*) &task_Contr);    /* Register the read access for label B */   
}
/*-----------------------------------------------------------*/

//void vLetContrTask_job(void) {
    /******** Init static var ********/
    

    /******** Main function ********/
    // mock control functions
    // Read Rotary Encoder angle, and send STOP signal to Control Variable for the Motor
    /*if (*ContrTask_Enc >= 170 && *ContrTask_Enc <= 190)     //STOP -- ~180
        (*task_Contr) = 0;
    else if(*ContrTask_Enc >= 80 && *ContrTask_Enc <= 100)  //LEFT -- ~90
        (*task_Contr) = 2;
    else if(*ContrTask_Enc >= 250 && *ContrTask_Enc <= 280) //RIGHT -- ~270 / -90
        (*task_Contr) = 3;
    else
        (*task_Contr) = 1;*/                                  //GO


    //printf("Deg in contr: %d\r\n", *ContrTask_Enc); //Read any inputs
//}
/*-----------------------------------------------------------*/

void vLetContrTask_job(void) {
    /******** Init static var ********/
    static bool first_time = true;
    static bool balance_on = false;
    /* CMSIS Variables */
    arm_pid_instance_a_f32 PID_Pend, PID_Rotor;
    float Deriv_Filt_Pend[2];
    float Deriv_Filt_Rotor[2];
    float Wo_t, fo_t, IWon_t;

    if(first_time){
        first_time = false;

        fo_t = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY;
        Wo_t = 2 * 3.141592654 * fo_t;
        IWon_t = 2 / (Wo_t * (T_Enc));
        Deriv_Filt_Pend[0] = 1 / (1 + IWon_t);
        Deriv_Filt_Pend[1] = Deriv_Filt_Pend[0] * (1 - IWon_t);

        fo_t = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR;
        Wo_t = 2 * 3.141592654 * fo_t;
        IWon_t = 2 / (Wo_t * (T_Motor));
        Deriv_Filt_Rotor[0] = 1 / (1 + IWon_t);
        Deriv_Filt_Rotor[1] = Deriv_Filt_Rotor[0] * (1 - IWon_t);

        *current_error_steps        = 0;
        *current_error_rotor_steps  = 0;
        PID_Pend.state_a[0] = 0;
        PID_Pend.state_a[1] = 0;
        PID_Pend.state_a[2] = 0;
        PID_Pend.state_a[3] = 0;
        PID_Pend.int_term   = 0;
        PID_Pend.control_output = 0;

        PID_Rotor.state_a[0]    = 0;
        PID_Rotor.state_a[1]    = 0;
        PID_Rotor.state_a[2]    = 0;
        PID_Rotor.state_a[3]    = 0;
        PID_Rotor.int_term      = 0;
        PID_Rotor.control_output = 0;

        encoder_angle_slope_corr_steps  = 0;
        pendulum_position_command_steps = 0;
        rotor_control_target_steps      = 0;
        rotor_position_steps            = 0;
        rotor_position_command_steps    = 0;
        feedforward_gain                = 1;
        encoder_position                = 0;   

        pid_filter_control_execute(&PID_Pend, current_error_steps, T_Enc, Deriv_Filt_Pend);

		pid_filter_control_execute(&PID_Rotor, current_error_rotor_steps, T_Motor, Deriv_Filt_Rotor);
    }

    /******** Main function ********/
    if (*ContrTask_Enc >= 178 && *ContrTask_Enc <= 182)
        balance_on = true;


    if(balance_on){
        encoder_position = *ContrTask_Enc;
        //encoder_position = count;   // steps/pulses instead of deg

        *current_error_steps = encoder_angle_slope_corr_steps
                + ENCODER_ANGLE_POLARITY * (encoder_position / ((float)(ENCODER_READ_ANGLE_SCALE/STEPPER_READ_POSITION_STEPS_PER_DEGREE)));

        pid_filter_control_execute(&PID_Pend, current_error_steps, T_Enc, Deriv_Filt_Pend);

    	//pid_filter_control_execute(&PID_Rotor, current_error_rotor_steps, T_Motor,  Deriv_Filt_Rotor);

		//rotor_control_target_steps = PID_Pend.control_output + PID_Rotor.control_output;
        rotor_control_target_steps = PID_Pend.control_output;

        //L6474_GoTo(0, rotor_control_target_steps/2);
        (*task_Contr) = (int32_t)(rotor_control_target_steps/2);
        printf("Target steps: %d\n", (int32_t)(rotor_control_target_steps/2));
    }
}
/*-----------------------------------------------------------*/