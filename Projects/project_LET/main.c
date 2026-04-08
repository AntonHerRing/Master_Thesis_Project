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
#define T_Enc   2
#define T_Motor 2
#define T_Contr 2
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
float rotor_position_command_steps_pf, rotor_position_command_steps_pf_prev;
int rotor_position_step_polarity;
float rotor_position_command_steps_prev;
float rotor_position_steps_prev, rotor_position_filter_steps, rotor_position_filter_steps_prev;


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

float* task_Enc;      /* Pointer to the local data of label ENC by Encoder task. */
float  task_Enc_data; /* Local copy of label ENC owned by LET Encoder task. */
float* PrintTask_Enc;      /* Pointer to the local data of label Enc by Print task. */
float  PrintTask_Enc_data; /* Local copy of label Enc owned by Print LET task. */
float* ContrTask_Enc;      /* Pointer to the local data of label ENC by Control task. */
float  ContrTask_Enc_data; /* Local copy of label ENC owned by Control LET task. */

int32_t* task_Motor;      /* Pointer to the local data of label Motor by Motor task. */
int32_t  task_Motor_data; /* Local copy of label Motor owned by LET Motor task. */
int32_t* PrintTask_Motor;      /* Pointer to the local data of label Motor by Print task. */
int32_t  PrintTask_Motor_data; /* Local copy of label Motor owned by Print LET task. */
int32_t* ContrTask_Motor;      /* Pointer to the local data of label ENC by Control task. */
int32_t  ContrTask_Motor_data; /* Local copy of label ENC owned by Control LET task. */

int32_t* task_Contr;      /* Pointer to the local data of label Contr by Control task. */
int32_t  task_Contr_data; /* Local copy of label Contr owned by LET Control task. */
int32_t* MotorTask_Contr;      /* Pointer to the local data of label Contr by Motor task. */
int32_t  MotorTask_Contr_data; /* Local copy of label Contr owned by Motor LET task. */
int32_t* PrintTask_Contr;      /* Pointer to the local data of label Motor by Print task. */
int32_t  PrintTask_Contr_data; /* Local copy of label Motor owned by Print LET task. */

// Rotary Encoder Interrupt Variables
volatile int32_t count = 0;
volatile int dir = 0;


/* Low pass filter variables */
float fo, Wo, IWon, iir_0, iir_1, iir_2;
float fo_LT, Wo_LT, IWon_LT;
float iir_LT_0, iir_LT_1, iir_LT_2;
float fo_s, Wo_s, IWon_s, iir_0_s, iir_1_s, iir_2_s;

// PID variables
/*struct PID Pid1;
struct PID *PID_Pend = &Pid1;

struct PID Pid2;
struct PID *PID_Rotor = &Pid2;*/

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
    trace_init();           /* Initialize the Tracing function*/
    

    //init_pid(PID_Pend, PID_Rotor);         /* Initialize PID variables with initial values*/
    
    if (xLetInit() == pdFALSE) {                    /* Initialize the LET module. */
        while (true);
    }

    // Initialize the Interrupts on the two A and B ports
    gpio_set_irq_enabled_with_callback(Phase_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled(Phase_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    /* Create a label and LET tasks that read/write from it. */
    xLetInitLabel("Enc", sizeof(float), &label_Enc, LET_COM_COPY);
    xLetInitLabel("Motor", sizeof(int32_t), &label_Motor, LET_COM_COPY);
    xLetInitLabel("Contr", sizeof(int32_t), &label_Contr, LET_COM_COPY);

    //low num = low prio, High num = high prio
    xLetTaskCreate(vLetEncTask_init, vLetEncTask_job, "LET_Enc_Task", 512, 5, T_Enc, T_Enc, 0, CORE0, &letEncTsk);
    xLetTaskCreate(vLetContrTask_init, vLetContrTask_job, "LET_Control_Task", 512, 4, T_Contr, T_Contr, 0, CORE0, &letContrTsk);
    xLetTaskCreate(vLetMotorTask_init, vLetMotorTask_job, "LET_Motor_Task", 512, 3, T_Motor, T_Motor, 0, CORE0, &letMotorTsk);
    xLetTaskCreate(vLetPrintTask_init, vLetPrintTask_job, "LET_Print_Task", 512, 2, T_Print, T_Print, 0, CORE0, &letPrintTsk);
    
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
    static int calibration_delay = 10;

    static float prev_value = 0;
    static float offset = 0;

    if(calibration_delay > 1)
        calibration_delay--;

    //Calibration step
    if((calibration_delay == 1) && (prev_value - get_encoder_steps(count)) == 0){
        if((get_encoder_steps(count) - offset) == 0)
            calibration_delay = 0;
        printf("Calibrating Encoder..");
        offset = get_encoder_steps(count);
        printf("Offset set at: %f\n", offset);

    }
    /******** Main function ********/
    if(calibration_delay != 0){
        prev_value = get_encoder_steps(count);
        //printf("Test Zero: %f\n", (prev_value - get_encoder_steps(count)));
    }
    else 
        (*task_Enc) = get_encoder_steps(count) - offset;
    
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
    static float desired_pos = 0.0;
    static float relativ_deg = 0.0;
    static float offset = 0.0;
    static bool first_time = true;

    static int32_t Contr_sig = 1;   // 1 == Go, -1 == Stop

    /******** Main function ********/
    motor_deg = (get_stepper_angle());
    (*task_Motor) = (int32_t)motor_deg; //write any inputs

    desired_pos = (float)(*MotorTask_Contr / MOTOR_STEPS_PER_DEGREE);

    if(first_time){
        first_time = false;
        printf("Calibrating Motor Position...\n");
        sleep_ms(20);
        L6474_SetHome(0, get_stepper_angle()* MOTOR_STEPS_PER_DEGREE);
        move_stepper_by(1.0);
        sleep_ms(20);
        move_stepper_by(-1.0);

        if(get_stepper_angle() != 0){
            printf("Incorrect Start Position::Stepper Not at 0. Recalibrating...\n");
            move_stepper_to(0);
        }
        else
            printf("Success! Stepper is positioned at 0\n");
        sleep_ms(10);
    }
    else if(abs(motor_deg) <= 270 && abs(desired_pos) <= 270){
        move_stepper_to(desired_pos);
    }
    else if(abs(motor_deg) > 270 && abs(desired_pos) > 270){
        printf("Error: Control task overshoot\n");
        L6474_HardStop(0);
    }
}
/*-----------------------------------------------------------*/

void vLetPrintTask_init(void) {
    PrintTask_Enc = &PrintTask_Enc_data;  /* Initialize the pointer to the local buffers */
    PrintTask_Motor = &PrintTask_Motor_data;
    PrintTask_Contr = &PrintTask_Contr_data;

    xLetTaskRegisterRead(&letPrintTsk, &label_Enc, (void*) &PrintTask_Enc);    /* Register the read access for label Enc */    
    xLetTaskRegisterRead(&letPrintTsk, &label_Motor, (void*) &PrintTask_Motor);    /* Register the read access for label Motor */   
    xLetTaskRegisterRead(&letPrintTsk, &label_Contr, (void*) &PrintTask_Contr);    /* Register the read access for label Control */  
}
/*-----------------------------------------------------------*/

void vLetPrintTask_job(void) {
    /******* Init static var *******/
    static bool first_time = true;
    static uint32_t run_time = 0; 
    

    /******** Main function ********/
    run_time += T_Print;

    //print data
    //printf("Run Time(s): %f\tDeg: %f\tMotor Deg: %d\tTarget Deg: %f\r\n", (float)run_time/1000.0,*PrintTask_Enc, *PrintTask_Motor, *PrintTask_Contr/STEPPER_READ_POSITION_STEPS_PER_DEGREE); //Read any inputs

    printf("Run Time(s): %f", (float)run_time/1000.0);
    printf("\t");
    printf("Deg: %f", *PrintTask_Enc);
    printf("\t");
    printf("Motor Deg: %d", *PrintTask_Motor);
    printf("\t");
    printf("Target Deg: %f", *PrintTask_Contr/STEPPER_READ_POSITION_STEPS_PER_DEGREE);
    printf("\r\n");
}
/*-----------------------------------------------------------*/

void vLetContrTask_init(void) {
    task_Contr = &task_Contr_data;  /* Initialize the pointer to the local buffers */
    ContrTask_Enc = &ContrTask_Enc_data;
    ContrTask_Motor = &ContrTask_Motor_data;

    xLetTaskRegisterRead(&letContrTsk, &label_Enc, (void*) &ContrTask_Enc);     /* Register the read access for label Enc */    
    xLetTaskRegisterWrite(&letContrTsk, &label_Contr, (void*) &task_Contr);     /* Register the write access for label Contr */   
    xLetTaskRegisterRead(&letContrTsk, &label_Motor, (void*) &ContrTask_Motor); /* Register the read access for label Motor */   
}

/*-----------------------------------------------------------*/

void vLetContrTask_job(void) {
    /******** Init static var ********/
    static bool first_time = true;
    static bool balance_on = false;
    /* CMSIS Variables */
    static arm_pid_instance_a_f32 PID_Pend, PID_Rotor;
    static float Deriv_Filt_Pend[2];
    static float Deriv_Filt_Rotor[2];
    static float Wo_t, fo_t, IWon_t;

    static float pend_period    = T_Enc / 1000.0;
    static float motor_period   = T_Motor / 1000.0;

    static float encoder_position_down;

    if(first_time){
        first_time = false;
        printf("Initiating Control Variables...\n");

        fo_t    = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY;
        Wo_t    = 2 * PI * fo_t;
        IWon_t  = 2 / (Wo_t * (pend_period));
        Deriv_Filt_Pend[0] = 1 / (1 + IWon_t);
        Deriv_Filt_Pend[1] = Deriv_Filt_Pend[0] * (1 - IWon_t);

        fo_t    = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR;
        Wo_t    = 2 * PI * fo_t;
        IWon_t  = 2 / (Wo_t * (motor_period));
        Deriv_Filt_Rotor[0] = 1 / (1 + IWon_t);
        Deriv_Filt_Rotor[1] = Deriv_Filt_Rotor[0] * (1 - IWon_t);

    	/* Compute Low Pass Filter Coefficients for Rotor Position filter and Encoder Angle Slope Correction */
        fo       = LP_CORNER_FREQ_ROTOR;
        Wo       = 2 * PI * fo;
        IWon     = 2 / (Wo * motor_period);
        iir_0    = 1 / (1 + IWon);
        iir_1    = iir_0;
        iir_2    = iir_0 * (1 - IWon);
        fo_s     = LP_CORNER_FREQ_STEP;
        Wo_s     = 2 * PI * fo_s;
        IWon_s   = 2 / (Wo_s * motor_period);
        iir_0_s  = 1 / (1 + IWon_s);
        iir_1_s  = iir_0_s;
        iir_2_s  = iir_0_s * (1 - IWon_s);
        fo_LT    = LP_CORNER_FREQ_LONG_TERM;
        Wo_LT    = 2 * PI * fo_LT;
        IWon_LT  = 2 / (Wo_LT * motor_period);
        iir_LT_0 = 1 / (1 + IWon_LT);
        iir_LT_1 = iir_LT_0;
        iir_LT_2 = iir_LT_0 * (1 - IWon_LT);

        current_error_steps         = malloc(sizeof(float));
        current_error_rotor_steps   = malloc(sizeof(float));
        *current_error_steps         = 0;
        *current_error_rotor_steps   = 0;

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

        PID_Pend.Kp = PRIMARY_PROPORTIONAL_MODE_1;
        PID_Pend.Ki = PRIMARY_INTEGRAL_MODE_1;
        PID_Pend.Kd = PRIMARY_DERIVATIVE_MODE_1;

        PID_Rotor.Kp = SECONDARY_PROPORTIONAL_MODE_1;
        PID_Rotor.Ki = SECONDARY_INTEGRAL_MODE_1;
        PID_Rotor.Kd = SECONDARY_DERIVATIVE_MODE_1;

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

        //other extra variable inits(maybe remove later)

        rotor_position_step_polarity = 1;
        rotor_position_command_steps_prev = 0;
        rotor_position_command_steps_pf_prev = 0;
        rotor_position_command_steps_pf = (float) ((rotor_position_step_polarity)
								* ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE
								* STEPPER_READ_POSITION_STEPS_PER_DEGREE);
        printf("rotor pos command: %f\n", rotor_position_command_steps_pf);

        //encoder_position_down           = *ContrTask_Enc;

        pid_filter_control_execute(&PID_Pend, current_error_steps, pend_period, Deriv_Filt_Pend);
		pid_filter_control_execute(&PID_Rotor, current_error_rotor_steps, motor_period, Deriv_Filt_Rotor);
    }

    /******** Main function ********/
    //if (abs(*ContrTask_Enc) >= (PI - 0.2) && abs(*ContrTask_Enc) <= (PI + 0.2))
    if (abs(*ContrTask_Enc) >= 1100 && abs(*ContrTask_Enc) <= 1300 && balance_on == false){
        balance_on = true;
        L6474_SetAnalogValue(0, L6474_TVAL, MAX_TORQUE_CONFIG);
    }
        

    if (balance_on){
        
        encoder_position = *ContrTask_Enc;

        //encoder_position = encoder_position_steps - encoder_position_down - (int)(180 * angle_scale);
        encoder_position -= (int)(180.0 * 1.0/(ENCODER_ANGLE_SCALE));
        //printf("Encoder position: %f\n",encoder_position);

        *current_error_steps = encoder_angle_slope_corr_steps
                + ENCODER_ANGLE_POLARITY * ((encoder_position/4.0) / ((float)(ENCODER_READ_ANGLE_SCALE/STEPPER_READ_POSITION_STEPS_PER_DEGREE)));

        pid_filter_control_execute(&PID_Pend, current_error_steps, pend_period, Deriv_Filt_Pend);

		/*rotor_position_command_steps = rotor_position_command_steps_pf * iir_0_s
				+ rotor_position_command_steps_pf_prev * iir_1_s
				- rotor_position_command_steps_prev * iir_2_s;
		rotor_position_command_steps_pf_prev = rotor_position_command_steps_pf;*/

        printf("rotor command step: %f\n", rotor_position_command_steps);
        //printf("iir_0_s: %f\tiir_1_s: %f\tiir_2_s: %f\n", iir_0_s, iir_1_s, iir_2_s);

        //*current_error_rotor_steps = rotor_position_filter_steps - rotor_position_command_steps;

    	//pid_filter_control_execute(&PID_Rotor, current_error_rotor_steps, motor_period,  Deriv_Filt_Rotor);

		//rotor_control_target_steps = PID_Pend.control_output + PID_Rotor.control_output;
        rotor_control_target_steps = PID_Pend.control_output;
        
        printf("Target steps: %f\tDec/2: %d\n", rotor_control_target_steps, (int32_t)(rotor_control_target_steps/2));

        (*task_Contr) = (int32_t)(rotor_control_target_steps/2);
        rotor_position_command_steps_prev = *MotorTask_Contr;   //record past data
        //printf("Enc pos: %f\t Target steps: %f\tCurr Error steps: %f\n", encoder_position, rotor_control_target_steps, *current_error_steps);
    }
    else
        encoder_position = 0;
}
/*-----------------------------------------------------------*/


// Scrapped functions
//previously in Control_job
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

    //Old Control_job
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