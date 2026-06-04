#ifndef TASKS_H
#define TASKS_H

#include <stdlib.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "bsp.h"
#include "let.h"
#include "trace.h"
#include "math.h"

#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/structs/io_bank0.h"

#include "Drivers/L16474/motor_rpi3b_interface.h"
#include "Drivers/L16474/l6474.h"
#include "Drivers/L16474/steppermotor.h"

#include "Control/Control.h"
#include "Encoder/Encoder.h"

#define T_Dummy  4      //2
#define T_Enc   2       //2
#define T_Motor 2       //2
#define T_Contr 5       //5
#define T_Print 10      //10
#define T_Btns  2       //2

#define ENC_OFFSET 1

#define CALC_ON         true
#define STEP_RESPONSE   false

/***** STM Var******/
extern float *current_error_steps, *current_error_rotor_steps;
extern float encoder_angle_slope_corr_steps;
extern float pendulum_position_command_steps;
extern float rotor_control_target_steps;
extern int rotor_position_steps;
extern float rotor_position_command_steps;
extern float feedforward_gain;
extern float encoder_position;
extern int rotor_position_step_polarity;
extern float rotor_position_command_steps_prev;
extern float rotor_position_steps_prev, rotor_position_filter_steps, rotor_position_filter_steps_prev;


//function definition
extern void L6474_StepClockHandler(uint8_t deviceId);

// LET Labels and Local communication variables

extern label_t label_Enc;        /* Data label used for LET tasks. */
extern label_t label_Motor;        /* Data label used for LET tasks. */
extern label_t label_Contr;        /* Data label used for LET tasks. */
extern label_t label_Btns;        /* Data label used for LET tasks. */

extern LetTask_t letEncTsk;    /*Handle for the LET rotary encoder task. */
extern LetTask_t letMotorTsk;  /*Handle for the LET stepper motor task. */
extern LetTask_t letContrTsk;  /*Handle for the LET Control task. */
extern LetTask_t letPrintTsk;  /*Handle for the LET Print task. */
extern LetTask_t letBtnsTsk;  /*Handle for the LET Buttons task. */
extern LetTask_t letDummyTsk;  /*Handle for  LET dummy task. */

extern float* task_Enc;      /* Pointer to the local data of label ENC by Encoder task. */
extern float  task_Enc_data; /* Local copy of label ENC owned by LET Encoder task. */
extern float* PrintTask_Enc;      /* Pointer to the local data of label Enc by Print task. */
extern float  PrintTask_Enc_data; /* Local copy of label Enc owned by Print LET task. */
extern float* ContrTask_Enc;      /* Pointer to the local data of label ENC by Control task. */
extern float  ContrTask_Enc_data; /* Local copy of label ENC owned by Control LET task. */

extern float* task_Motor;      /* Pointer to the local data of label Motor by Motor task. */
extern float  task_Motor_data; /* Local copy of label Motor owned by LET Motor task. */
extern float* PrintTask_Motor;      /* Pointer to the local data of label Motor by Print task. */
extern float  PrintTask_Motor_data; /* Local copy of label Motor owned by Print LET task. */
extern float* ContrTask_Motor;      /* Pointer to the local data of label ENC by Control task. */
extern float  ContrTask_Motor_data; /* Local copy of label ENC owned by Control LET task. */

extern float* task_Contr;      /* Pointer to the local data of label Contr by Control task. */
extern float  task_Contr_data; /* Local copy of label Contr owned by LET Control task. */
extern float* MotorTask_Contr;      /* Pointer to the local data of label Contr by Motor task. */
extern float  MotorTask_Contr_data; /* Local copy of label Contr owned by Motor LET task. */
extern float* PrintTask_Contr;      /* Pointer to the local data of label Motor by Print task. */
extern float  PrintTask_Contr_data; /* Local copy of label Motor owned by Print LET task. */

extern int16_t* task_Btns;      /* Pointer to the local data of label Btns by Buttons task. */
extern int16_t  task_Btns_data; /* Local copy of label Butns owned by LET Buttons task. */
extern int16_t* MotorTask_Btns;      /* Pointer to the local data of label Btns by Buttons task. */
extern int16_t  MotorTask_Btns_data; /* Local copy of label Butns owned by LET Buttons task. */
extern int16_t* EncTask_Btns;      /* Pointer to the local data of label Btns by Buttons task. */
extern int16_t  EncTask_Btns_data; /* Local copy of label Butns owned by LET Buttons task. */

// Rotary Encoder Interrupt Variables
extern volatile int32_t count;
extern volatile int dir;

/* Low pass filter variables */
extern float fo, Wo, IWon, iir_0, iir_1, iir_2;
extern float fo_LT, Wo_LT, IWon_LT;
extern float iir_LT_0, iir_LT_1, iir_LT_2;
extern float fo_s, Wo_s, IWon_s, iir_0_s, iir_1_s, iir_2_s;

/******** Init Control var ********/
extern bool first_time;
extern bool balance_on;

/* CMSIS Variables */
extern inverted_pid_contr PID_Pend, PID_Rotor;
extern float Deriv_Filt_Pend[3];
extern float Deriv_Filt_Rotor[3];
extern float Wo_t, fo_t, IWon_t;

extern float pend_period;
extern float motor_period;
extern float contr_period;

extern float encoder_position_down;


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

/**
 * @brief Initialization function of Buttons LET task.
 */
void vLetBtnsTask_init(void);

/**
 * @brief Job function of Buttons LET task.
 */
void vLetBtnsTask_job(void);

/**
 * @brief Initialization function of Dummy LET task.
 */
void vLetDummyTask_init(void);

/**
 * @brief Job function of Dummy LET task.
 */
void vLetDummyTask_job(void);

#endif
