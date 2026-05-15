//#pragma GCC optimize ("O0") /* Incldue for dubuggning. Easier viewing of variables */


#include <stdio.h>
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
/*#define T_Enc   100
#define T_Motor 50
#define T_Contr 100
#define T_Print 100*/

//2
#define T_Enc   2//3//2
#define T_Motor 2
#define T_Contr 2//5//2   //2
#define T_Print 25  //50
#define T_Btns  2

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
float rotor_position_command_steps_pf, rotor_position_command_steps_pf_prev;
int rotor_position_step_polarity;
float rotor_position_command_steps_prev;
float rotor_position_steps_prev, rotor_position_filter_steps, rotor_position_filter_steps_prev;
int i;
int impulse_start_index;
int angle_cal_complete;
int chirp_cycle;


//function definition
extern void L6474_StepClockHandler(uint8_t deviceId);

// LET Labels and Local communication variables

label_t label_Enc;        /* Data label used for LET tasks. */
label_t label_Motor;        /* Data label used for LET tasks. */
label_t label_Contr;        /* Data label used for LET tasks. */
label_t label_Btns;        /* Data label used for LET tasks. */

LetTask_t letEncTsk;    /*Handle for the LET rotary encoder task. */
LetTask_t letMotorTsk;  /*Handle for the LET stepper motor task. */
LetTask_t letContrTsk;  /*Handle for the LET Control task. */
LetTask_t letPrintTsk;  /*Handle for the LET Print task. */
LetTask_t letBtnsTsk;  /*Handle for the LET Buttons task. */

float* task_Enc;      /* Pointer to the local data of label ENC by Encoder task. */
float  task_Enc_data; /* Local copy of label ENC owned by LET Encoder task. */
float* PrintTask_Enc;      /* Pointer to the local data of label Enc by Print task. */
float  PrintTask_Enc_data; /* Local copy of label Enc owned by Print LET task. */
float* ContrTask_Enc;      /* Pointer to the local data of label ENC by Control task. */
float  ContrTask_Enc_data; /* Local copy of label ENC owned by Control LET task. */

float* task_Motor;      /* Pointer to the local data of label Motor by Motor task. */
float  task_Motor_data; /* Local copy of label Motor owned by LET Motor task. */
float* PrintTask_Motor;      /* Pointer to the local data of label Motor by Print task. */
float  PrintTask_Motor_data; /* Local copy of label Motor owned by Print LET task. */
float* ContrTask_Motor;      /* Pointer to the local data of label ENC by Control task. */
float  ContrTask_Motor_data; /* Local copy of label ENC owned by Control LET task. */

float* task_Contr;      /* Pointer to the local data of label Contr by Control task. */
float  task_Contr_data; /* Local copy of label Contr owned by LET Control task. */
float* MotorTask_Contr;      /* Pointer to the local data of label Contr by Motor task. */
float  MotorTask_Contr_data; /* Local copy of label Contr owned by Motor LET task. */
float* PrintTask_Contr;      /* Pointer to the local data of label Motor by Print task. */
float  PrintTask_Contr_data; /* Local copy of label Motor owned by Print LET task. */

int16_t* task_Btns;      /* Pointer to the local data of label Btns by Buttons task. */
int16_t  task_Btns_data; /* Local copy of label Butns owned by LET Buttons task. */
int16_t* MotorTask_Btns;      /* Pointer to the local data of label Btns by Buttons task. */
int16_t  MotorTask_Btns_data; /* Local copy of label Butns owned by LET Buttons task. */
int16_t* EncTask_Btns;      /* Pointer to the local data of label Btns by Buttons task. */
int16_t  EncTask_Btns_data; /* Local copy of label Butns owned by LET Buttons task. */

// Rotary Encoder Interrupt Variables
volatile int32_t count = 0;
volatile int dir = 0;


/* Low pass filter variables */
float fo, Wo, IWon, iir_0, iir_1, iir_2;
float fo_LT, Wo_LT, IWon_LT;
float iir_LT_0, iir_LT_1, iir_LT_2;
float fo_s, Wo_s, IWon_s, iir_0_s, iir_1_s, iir_2_s;

/******** Init Control var ********/
bool first_time = true;
bool balance_on = false;

/* CMSIS Variables */
/*arm_pid_instance_a_f32 PID_Pend, PID_Rotor;
float Deriv_Filt_Pend[3];
float Deriv_Filt_Rotor[3];*/

inverted_pid_contr PID_Pend, PID_Rotor;

float Wo_t, fo_t, IWon_t;

float pend_period    = T_Enc / 1000.0;
float motor_period   = T_Motor / 1000.0;
float contr_period   = T_Contr / 1000.0;

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

// Hook for detective stack overflow
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName ){
    char taskname = *pcTaskName;

    TaskHandle_t task = xTask;

    printf("Warning: The task %s has a stack Overflow!\n");
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
    sleep_ms(1000);
    init_rotary_encoder();  /* Initialize the Rotary Encoder. */
    init_motor();           /* Initialize the Stepper Motor*/
    trace_init();           /* Initialize the Tracing function*/
    
    if (xLetInit() == pdFALSE) {                    /* Initialize the LET module. */
        while (true);
    }

    /* Create a label and LET tasks that read/write from it. */
    xLetInitLabel("Enc", sizeof(float), &label_Enc, LET_COM_COPY);
    xLetInitLabel("Motor", sizeof(float), &label_Motor, LET_COM_COPY);
    xLetInitLabel("Contr", sizeof(float), &label_Contr, LET_COM_COPY);
    xLetInitLabel("Btns", sizeof(int16_t), &label_Btns, LET_COM_COPY);

    //low num = low prio, High num = high prio
    xLetTaskCreate(vLetEncTask_init, vLetEncTask_job, "LET_Enc_Task", 512, 6, T_Enc, T_Enc, 0, CORE1, &letEncTsk);
    xLetTaskCreate(vLetContrTask_init, vLetContrTask_job, "LET_Control_Task", 5120, 5, T_Contr, T_Contr, 0, CORE0, &letContrTsk);
    xLetTaskCreate(vLetBtnsTask_init, vLetBtnsTask_job, "LET_Buttons_Task", 512, 4, T_Btns, T_Btns, 0, CORE0, &letBtnsTsk);
    xLetTaskCreate(vLetMotorTask_init, vLetMotorTask_job, "LET_Motor_Task", 18216, 3, T_Motor, T_Motor, 0, CORE0, &letMotorTsk);    //18216          //10240, is too much
    xLetTaskCreate(vLetPrintTask_init, vLetPrintTask_job, "LET_Print_Task", 1024, 2, T_Print, T_Print, 0, CORE0, &letPrintTsk);
    
    vTaskStartScheduler();  /* Start the scheduler. */
    
    while (true) { 
        sleep_ms(1000); /* Should not reach here... */
    }
}

/*-----------------------------------------------------------*/

void vLetBtnsTask_init(void) {
    task_Btns = &task_Btns_data;    /* Initialize the pointer to the local buffer for label BTNS */

    /******** Register LET variables ********/
    xLetTaskRegisterWrite(&letBtnsTsk, &label_Btns, (void*) &task_Btns);    /* Register the write access for label A */    
}
/*-----------------------------------------------------------*/

void vLetBtnsTask_job(void) {
    //[0] == btn1 == 1, [1] == btn2 == 2, [2] == btn3 == 4, [3] == btn4 == 8
    uint8_t btn1 = BSP_GetInput(SW_5);
    uint8_t btn2 = BSP_GetInput(SW_6);
    uint8_t btn3 = BSP_GetInput(SW_7);
    uint8_t btn4 = BSP_GetInput(SW_8);

    uint8_t buttons = 0x0 | (!btn1 | (!btn2 << 1) | (!btn3 << 2) | (!btn4 << 3));

    *task_Btns = buttons;
}

/*-----------------------------------------------------------*/

void vLetEncTask_init(void) {
    task_Enc = &task_Enc_data;    /* Initialize the pointer to the local buffer for label ENC */
    EncTask_Btns = &EncTask_Btns_data;

    // Initialize the Interrupts on the two A and B ports
    gpio_set_irq_enabled_with_callback(Phase_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled(Phase_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    /******** Register LET variables ********/
    xLetTaskRegisterWrite(&letEncTsk, &label_Enc, (void*) &task_Enc);           /* Register the write access for label Enc */   
    xLetTaskRegisterWrite(&letBtnsTsk, &label_Btns, (void*) &EncTask_Btns);     /* Register the read access for label Btns */   
}
/*-----------------------------------------------------------*/

void vLetEncTask_job(void) {
    /******** Init static var ********/

    uint32_t current_time = xTaskGetTickCount();

    //Handle button inputs
    switch(*MotorTask_Btns){
        case 4: count = 0; break; //reset pendulum angle
        default: break;
    }
    
    (*task_Enc) = get_encoder_angle_continous(count);

    //(*task_Enc) = step_response_enc(current_time, 8000);

    
}
/*-----------------------------------------------------------*/

void vLetMotorTask_init(void) {
    task_Motor = &task_Motor_data;    /* Initialize the pointer to the local buffer for label Motor */
    MotorTask_Contr = &MotorTask_Contr_data;

    /******** Calibrate Motor ********/
    sleep_ms(10);
    move_stepper_by(1.0);
    sleep_ms(10);
    move_stepper_by(-1.0);
    L6474_SetHome(0, (int32_t)(get_stepper_angle()*MOTOR_STEPS_PER_DEGREE));
    sleep_ms(20);

    /******** Register LET variables ********/
    xLetTaskRegisterWrite(&letMotorTsk, &label_Motor, (void*) &task_Motor);    /* Register the write access for label Motor */   
    xLetTaskRegisterRead(&letMotorTsk, &label_Contr, (void*) &MotorTask_Contr); /* Register the read access for label Contr */ 
    xLetTaskRegisterRead(&letMotorTsk, &label_Btns, (void*) &MotorTask_Btns);    /* Register the write access for label Motor */ 
}
/*-----------------------------------------------------------*/

void vLetMotorTask_job(void) {
    /******** Init static var ********/
    static float motor_deg = 0.0;
    static float desired_pos = 0.0;

    static int16_t buttons = 0;
    static bool pos_overflow = false;
    static float collector = 0;
    /******** Main function ********/
    //Handle button inputs
    buttons = *MotorTask_Btns;
    switch(buttons){
        case 1: collector += 0.2; break;    //0.2
        case 2: collector -= 0.2; break;
        case 4:
            collector = 0;
            L6474_SetHome(0, get_stepper_angle()* MOTOR_STEPS_PER_DEGREE);
        break;
        case 8: pos_overflow = true; break;
        default: break;
    }

    //Read and write motor position values
    desired_pos = *MotorTask_Contr - collector;
    motor_deg = get_stepper_angle();
    (*task_Motor) = motor_deg; //write any inputs

    //printf("Desired pos: %f\n", desired_pos);

    // Catch Control signal overflow
    if(!pos_overflow && abs(motor_deg) >= 360 || abs(desired_pos) >= 360){
        //L6474_HardStop(0);
        pos_overflow = true;
    }
    else if (pos_overflow && round(motor_deg) == round(desired_pos))    //go again when stabilized
        pos_overflow = false;

    // Move if signal is stable
    if(!pos_overflow){
        move_stepper_to(desired_pos);
    }
}
/*-----------------------------------------------------------*/

void vLetPrintTask_init(void) {
    PrintTask_Enc = &PrintTask_Enc_data;  /* Initialize the pointer to the local buffers */
    PrintTask_Motor = &PrintTask_Motor_data;
    PrintTask_Contr = &PrintTask_Contr_data;

    /******** Register LET variables ********/
    xLetTaskRegisterRead(&letPrintTsk, &label_Enc, (void*) &PrintTask_Enc);    /* Register the read access for label Enc */    
    xLetTaskRegisterRead(&letPrintTsk, &label_Motor, (void*) &PrintTask_Motor);    /* Register the read access for label Motor */   
    xLetTaskRegisterRead(&letPrintTsk, &label_Contr, (void*) &PrintTask_Contr);    /* Register the read access for label Control */  
}
/*-----------------------------------------------------------*/

void vLetPrintTask_job(void) {
    /******* Init static var *******/
    static uint32_t run_time = 0; 

    /******** Main function ********/
    run_time += T_Print;

    //print data
    printf("#-42-#: Run Time(s): %f\tDeg: %f\tMotor Deg: %f\tTarget Deg: %f\tEnd\r\n", 
            (float)run_time/1000.0,*PrintTask_Enc, *PrintTask_Motor, *PrintTask_Contr); //Read any inputs
}
/*-----------------------------------------------------------*/

void vLetContrTask_init(void) {
    task_Contr = &task_Contr_data;  /* Initialize the pointer to the local buffers */
    ContrTask_Enc = &ContrTask_Enc_data;
    ContrTask_Motor = &ContrTask_Motor_data;

    fo_t    = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY;
    Wo_t    = 2 * PI * fo_t;
    IWon_t  = 2 / (Wo_t * (contr_period));
    PID_Pend.ff_gain = 1 / (1 + IWon_t);
    PID_Pend.fb_gain = PID_Pend.ff_gain * (1 - IWon_t);
    PID_Pend.tau = 1.0f / Wo_t;
    //Deriv_Filt_Pend[0] = 1 / (1 + IWon_t);
    //Deriv_Filt_Pend[1] = Deriv_Filt_Pend[0] * (1 - IWon_t);

    fo_t    = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR;
    Wo_t    = 2 * PI * fo_t;
    IWon_t  = 2 / (Wo_t * (contr_period));
    PID_Rotor.ff_gain = 1 / (1 + IWon_t);
    PID_Rotor.fb_gain = PID_Rotor.ff_gain * (1 - IWon_t);
    PID_Rotor.tau = 1.0f / Wo_t;
    //Deriv_Filt_Rotor[0] = 1 / (1 + IWon_t);
    //Deriv_Filt_Rotor[1] = Deriv_Filt_Rotor[0] * (1 - IWon_t);

    current_error_steps         = malloc(sizeof(float));
    current_error_rotor_steps   = malloc(sizeof(float));
    *current_error_steps         = 0;
    *current_error_rotor_steps   = 0;

    /*PID_Pend.state_a[0] = 0;
    PID_Pend.state_a[1] = 0;
    PID_Pend.state_a[2] = 0;
    PID_Pend.state_a[3] = 0;
    PID_Pend.state_a[4] = 0;*/

    PID_Pend.Set_point  = 0;
    PID_Pend.measurment = 0;

    PID_Pend.prev_measurment = 0;
    PID_Pend.prev_error_1    = 0;
    PID_Pend.prev_error_2    = 0;
    PID_Pend.prev_diff       = 0;
    PID_Pend.prev_filt       = 0;

    PID_Pend.tau             = 0;

    PID_Pend.int_term        = 0;
    PID_Pend.control_output  = 0;

    /*PID_Rotor.state_a[0]    = 0;
    PID_Rotor.state_a[1]    = 0;
    PID_Rotor.state_a[2]    = 0;
    PID_Rotor.state_a[3]    = 0;
    PID_Rotor.state_a[4]    = 0;*/
    PID_Rotor.Set_point  = 0;
    PID_Rotor.measurment = 0;

    PID_Rotor.prev_measurment = 0;
    PID_Rotor.prev_error_1    = 0;
    PID_Rotor.prev_error_2    = 0;
    PID_Rotor.prev_diff       = 0;
    PID_Rotor.prev_filt       = 0;

    PID_Rotor.tau             = 0;
    
    PID_Rotor.int_term        = 0;
    PID_Rotor.control_output  = 0;

    PID_Pend.Kp = PRIMARY_PROPORTIONAL_MODE_1;
    PID_Pend.Ki = PRIMARY_INTEGRAL_MODE_1;
    PID_Pend.Kd = PRIMARY_DERIVATIVE_MODE_1;

    PID_Rotor.Kp = SECONDARY_PROPORTIONAL_MODE_1;
    PID_Rotor.Ki = SECONDARY_INTEGRAL_MODE_1;
    PID_Rotor.Kd = SECONDARY_DERIVATIVE_MODE_1;

    PID_Pend.Set_point  = 180 * STEPPER_READ_POSITION_STEPS_PER_DEGREE;     //180
    PID_Rotor.Set_point = 0;     //0     //70

    encoder_angle_slope_corr_steps  = 0;
    pendulum_position_command_steps = 0;
    rotor_control_target_steps      = 0;
    rotor_position_steps            = 0;
    rotor_position_command_steps    = 0;
    feedforward_gain                = 1;
    encoder_position                = 0; 

    rotor_position_steps_prev        = 0;
	rotor_position_filter_steps      = 0;
	rotor_position_filter_steps_prev = 0;

    i = 0;
    impulse_start_index = 0;
    angle_cal_complete = 0;
    chirp_cycle = 0;

    //other extra variable inits(maybe remove later)
	/* Compute Low Pass Filter Coefficients for Rotor Position filter and Encoder Angle Slope Correction */
	/*fo = LP_CORNER_FREQ_ROTOR;
	Wo = 2 * 3.141592654 * fo;
	IWon = 2 / (Wo * contr_period);
	iir_0 = 1 / (1 + IWon);
	iir_1 = iir_0;
	iir_2 = iir_0 * (1 - IWon);
	fo_s = LP_CORNER_FREQ_STEP;
	Wo_s = 2 * 3.141592654 * fo_s;
	IWon_s = 2 / (Wo_s * contr_period);
	iir_0_s = 1 / (1 + IWon_s);
	iir_1_s = iir_0_s;
	iir_2_s = iir_0_s * (1 - IWon_s);
	fo_LT = LP_CORNER_FREQ_LONG_TERM;
	Wo_LT = 2 * 3.141592654 * fo_LT;
	IWon_LT = 2 / (Wo_LT * contr_period);
	iir_LT_0 = 1 / (1 + IWon_LT);
	iir_LT_1 = iir_LT_0;
	iir_LT_2 = iir_LT_0 * (1 - IWon_LT);*/

    rotor_position_step_polarity = 1;
    rotor_position_command_steps_prev = 0;
    rotor_position_command_steps_pf_prev = 0;
    rotor_position_command_steps_pf = (float) ((rotor_position_step_polarity)
							* ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE
							* STEPPER_READ_POSITION_STEPS_PER_DEGREE);

    pid_filter_control_execute(&PID_Pend, current_error_steps, pend_period);
	pid_filter_control_execute(&PID_Rotor, current_error_rotor_steps, motor_period);
    //pid_filter_control_executeV2(&PID_Pend, current_error_steps, pend_period, DERIVATIVE_LOW_PASS_CORNER_FREQUENCY);
    //pid_filter_control_executeV2(&PID_Rotor, current_error_rotor_steps, motor_period, DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR);

    //pid_filter_control_execute_Incremental(&PID_Pend, current_error_steps, pend_period, Deriv_Filt_Pend);
    //pid_filter_control_execute_Incremental(&PID_Rotor, current_error_rotor_steps, motor_period, Deriv_Filt_Rotor);

    xLetTaskRegisterRead(&letContrTsk, &label_Enc, (void*) &ContrTask_Enc);     /* Register the read access for label Enc */    
    xLetTaskRegisterWrite(&letContrTsk, &label_Contr, (void*) &task_Contr);     /* Register the write access for label Contr */   
    xLetTaskRegisterRead(&letContrTsk, &label_Motor, (void*) &ContrTask_Motor); /* Register the read access for label Motor */   
}

/*-----------------------------------------------------------*/

void vLetContrTask_job(void) {
    static float Polarity = -1; //-1
    static float bias = 0;

    /******** Main function ********/
    if (abs(*ContrTask_Enc) >= 179.5 && abs(*ContrTask_Enc) <= 180.5 && balance_on == false){
        balance_on = true;
        L6474_SetAnalogValue(0, L6474_TVAL, MAX_TORQUE_CONFIG);
    }
   
    if (balance_on && (*ContrTask_Enc > 150 &&  *ContrTask_Enc < 210)){   		
            
        PID_Pend.measurment = *ContrTask_Enc * STEPPER_READ_POSITION_STEPS_PER_DEGREE;
        //encoder_position =  Pend_target - *ContrTask_Enc;
        encoder_position =  PID_Pend.Set_point - PID_Pend.measurment;

		/* Compute Low Pass Filtered rotor position difference */

		/*rotor_position_diff_prev = rotor_position_diff;

		if (enable_disturbance_rejection_step == 0){
			rotor_position_diff = rotor_position_filter_steps
					- rotor_position_command_steps;
		}
		if (enable_disturbance_rejection_step == 1){
			rotor_position_diff = rotor_position_filter_steps;
		}*/


		/*
		 * Compute error between Pendulum Angle and Pendulum Tracking Angle in units of steps
		* Apply scale factor to match angle to step gain of rotor actuator
		*
		*/

		//*current_error_steps = ENCODER_ANGLE_POLARITY * (encoder_position / 
        //                      STEPPER_READ_POSITION_STEPS_PER_DEGREE);
        *current_error_steps = ENCODER_ANGLE_POLARITY * encoder_position;

		pid_filter_control_execute(&PID_Pend, current_error_steps, contr_period);

		/* Acquire rotor position and compute low pass filtered rotor position */

        //rotor_position_steps = *ContrTask_Motor * STEPPER_READ_POSITION_STEPS_PER_DEGREE;
        PID_Rotor.measurment = *ContrTask_Motor * STEPPER_READ_POSITION_STEPS_PER_DEGREE;

		/* Optional rotor position filter */

		/*rotor_position_filter_steps = (float) (rotor_position_steps) * iir_0
				+ rotor_position_steps_prev * iir_1
				- rotor_position_filter_steps_prev * iir_2;
		rotor_position_steps_prev = (float) (rotor_position_steps);
		rotor_position_filter_steps_prev = rotor_position_filter_steps;


		rotor_position_filter_steps = rotor_position_steps;*/


		/*  Create rotor angle reference tracking impulse signal 
		if (ENABLE_ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE == 1 && i != 0) {
			if ((i % ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL) == 0) {
				rotor_position_command_steps =
						(float) (ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE
								* STEPPER_READ_POSITION_STEPS_PER_DEGREE);
				impulse_start_index = 0;
			}
			if (impulse_start_index
					> ROTOR_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD) {
				rotor_position_command_steps = 0;
			}
			impulse_start_index++;
		}
        /**/

		/*
		 * Create pendulum angle reference tracking impulse signal.  Polarity of impulse alternates 
		if (i != 0) {
			if ((i % PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL) == 0) {
				pendulum_position_command_steps =
						(float) (PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE
								/PENDULUM_POSITION_IMPULSE_AMPLITUDE_SCALE);
				//chirp_cycle = 0;
				impulse_start_index = 0;
			}
			if (impulse_start_index
					> PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD) {
				pendulum_position_command_steps = 0;
			}
			impulse_start_index++;
			//chirp_cycle++;
		}
        /**/

		/*  Create rotor angle reference tracking  step signal 
		if ((i % ROTOR_POSITION_STEP_RESPONSE_CYCLE_INTERVAL) == 0) {
			rotor_position_step_polarity = -rotor_position_step_polarity;
		}
			
		rotor_position_command_steps_pf = (float) ((rotor_position_step_polarity)
				* ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE
				* STEPPER_READ_POSITION_STEPS_PER_DEGREE);
        /**/

        *current_error_rotor_steps = PID_Rotor.Set_point - PID_Rotor.measurment;

        pid_filter_control_execute(&PID_Rotor, current_error_rotor_steps, contr_period);


		rotor_control_target_steps = PID_Pend.control_output + PID_Rotor.control_output;
        rotor_control_target_steps /= STEPPER_READ_POSITION_STEPS_PER_DEGREE;
    
        (*task_Contr) = rotor_control_target_steps;
        i++;
        
        //printf("i: %d\tPend_com_step %f\tRotor_com_step %f\tfilr_rotor_stps: %f\n", i, pendulum_position_command_steps, rotor_position_command_steps, rotor_position_filter_steps);
        //printf("Enc pos: %f\t Target steps: %f\tCurr Error steps: %f\n", encoder_position, rotor_control_target_steps, *current_error_steps);
    }
    else if (!balance_on) 
        (*task_Contr) = 0;
}
/*-----------------------------------------------------------*/
