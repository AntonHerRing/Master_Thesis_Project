#include <stdio.h>
#include "tasks.h"

label_t label_Enc;        /* Data label used for LET tasks. */
label_t label_Motor;        /* Data label used for LET tasks. */
label_t label_Contr;        /* Data label used for LET tasks. */
label_t label_Btns;        /* Data label used for LET tasks. */

LetTask_t letEncTsk;    /*Handle for the LET rotary encoder task. */
LetTask_t letMotorTsk;  /*Handle for the LET stepper motor task. */
LetTask_t letContrTsk;  /*Handle for the LET Control task. */
LetTask_t letPrintTsk;  /*Handle for the LET Print task. */
LetTask_t letBtnsTsk;  /*Handle for the LET Buttons task. */
LetTask_t letDummyTsk;  /*Handle for  LET dummy task. */

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

/***** STM Var******/
float *current_error_steps, *current_error_rotor_steps;
float rotor_control_target_steps;
float encoder_position;

//function definition
extern void L6474_StepClockHandler(uint8_t deviceId);

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
inverted_pid_contr PID_Pend, PID_Rotor;
float Wo_t, fo_t, IWon_t;

float pend_period    = T_Enc / 1000.0;
float motor_period   = T_Motor / 1000.0;
float contr_period   = T_Contr / 1000.0;

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
    xLetTaskRegisterRead(&letBtnsTsk, &label_Btns, (void*) &EncTask_Btns);     /* Register the read access for label Btns */   
}
/*-----------------------------------------------------------*/

void vLetEncTask_job(void) {
    /******** Init static var ********/

    uint32_t current_time = xTaskGetTickCount();

    //Handle button inputs
    switch(*EncTask_Btns){
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

    // Catch Control signal overflow
    if(!pos_overflow && abs(motor_deg) >= 360 || abs(desired_pos) >= 360){
        L6474_HardStop(0);
        pos_overflow = true;
    }

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
    static uint32_t run_time    = 0; 
    static float inc_mean       = 0;
    static float past_inc_mean  = 0;

    static float inc_variance   = 0;
    static float inc_stndDev    = 0;
    static float samples        = 0;

    static bool activate_calc = CALC_ON;
    static bool set_point_reached = false;

    /******** Main function ********/
    run_time += T_Print;

    //print data
    printf("#-42-#: Run Time(s): %f\tDeg: %f\tMotor Deg: %f\tTarget Deg: %f\tEnd\r\n", 
            (float)run_time/1000.0,*PrintTask_Enc, *PrintTask_Motor, *PrintTask_Contr); //Read any inputs

    if(!set_point_reached && (int)(*PrintTask_Enc) == 180 ){
        set_point_reached = true;
    }

    /* Calculate Incremental mean, standard deviation and Variance*/
    if (activate_calc && set_point_reached){
        samples++;
        past_inc_mean = inc_mean;
        inc_mean = inc_mean + (*PrintTask_Enc - inc_mean)/samples;

        inc_variance = ((samples - 2.0f)*inc_variance + (samples - 1.0f)
                      * (past_inc_mean - inc_mean)*(past_inc_mean - inc_mean)
                      + (*PrintTask_Enc - inc_mean)*(*PrintTask_Enc - inc_mean))
                      / (samples - 1.0f);

        inc_stndDev = sqrt(inc_variance);
        printf("#-32-#: Samples: %f\tMean %f\tVariance: %f\tStandard Deviation; %f\n", samples, inc_mean, inc_variance, inc_stndDev);
    }

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

    fo_t    = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR;
    Wo_t    = 2 * PI * fo_t;
    IWon_t  = 2 / (Wo_t * (contr_period));
    PID_Rotor.ff_gain = 1 / (1 + IWon_t);
    PID_Rotor.fb_gain = PID_Rotor.ff_gain * (1 - IWon_t);
    PID_Rotor.tau = 1.0f / Wo_t;

    current_error_steps         = malloc(sizeof(float));
    current_error_rotor_steps   = malloc(sizeof(float));
    *current_error_steps         = 0;
    *current_error_rotor_steps   = 0;


    PID_Pend.Set_point  = 0;
    PID_Pend.measurment = 0;

    PID_Pend.prev_measurment = 0;
    PID_Pend.prev_error_1    = 0;
    PID_Pend.prev_error_2    = 0;
    PID_Pend.prev_diff       = 0;
    PID_Pend.prev_filt       = 0;
    PID_Pend.prev_set_point  = 0;

    PID_Pend.int_term        = 0;
    PID_Pend.control_output  = 0;

    PID_Pend.clamp_on        = false;

    PID_Rotor.Set_point  = 0;
    PID_Rotor.measurment = 0;

    PID_Rotor.prev_measurment = 0;
    PID_Rotor.prev_error_1    = 0;
    PID_Rotor.prev_error_2    = 0;
    PID_Rotor.prev_diff       = 0;
    PID_Rotor.prev_filt       = 0;
	PID_Rotor.prev_set_point  = 0;
    
    PID_Rotor.int_term        = 0;
    PID_Rotor.control_output  = 0;

    PID_Rotor.clamp_on        = true;

    PID_Pend.Kp = PRIMARY_PROPORTIONAL_MODE_1;
    PID_Pend.Ki = PRIMARY_INTEGRAL_MODE_1;
    PID_Pend.Kd = PRIMARY_DERIVATIVE_MODE_1;

    PID_Rotor.Kp = SECONDARY_PROPORTIONAL_MODE_1;
    PID_Rotor.Ki = SECONDARY_INTEGRAL_MODE_1;
    PID_Rotor.Kd = SECONDARY_DERIVATIVE_MODE_1;

    PID_Pend.Set_point  = 180 * STEPPER_READ_POSITION_STEPS_PER_DEGREE;     //180
    PID_Rotor.Set_point = 0  * STEPPER_READ_POSITION_STEPS_PER_DEGREE;     //0     //70

    rotor_control_target_steps      = 0;
    encoder_position                = 0; 

    xLetTaskRegisterRead(&letContrTsk, &label_Enc, (void*) &ContrTask_Enc);     /* Register the read access for label Enc */    
    xLetTaskRegisterWrite(&letContrTsk, &label_Contr, (void*) &task_Contr);     /* Register the write access for label Contr */   
    xLetTaskRegisterRead(&letContrTsk, &label_Motor, (void*) &ContrTask_Motor); /* Register the read access for label Motor */   
}

/*-----------------------------------------------------------*/

void vLetContrTask_job(void) {
    float lambda = 0.87;//0.82; //0.91;

    /******** Main function ********/
    /* Activate Balancing*/
    if (abs(*ContrTask_Enc) >= 179.5 && abs(*ContrTask_Enc) <= 180.5 && balance_on == false){
        balance_on = true;
        L6474_SetAnalogValue(0, L6474_TVAL, MAX_TORQUE_CONFIG);
    }
   
    if (balance_on && (*ContrTask_Enc > 140 &&  *ContrTask_Enc < 220)){
        /* Calculate Rotor SP - PV*/
        PID_Rotor.measurment = *ContrTask_Motor * STEPPER_READ_POSITION_STEPS_PER_DEGREE;
        *current_error_rotor_steps = PID_Rotor.Set_point - PID_Rotor.measurment;

        pid_filter_control_execute(&PID_Rotor, current_error_rotor_steps, contr_period);

        PID_Pend.measurment = *ContrTask_Enc * STEPPER_READ_POSITION_STEPS_PER_DEGREE;
        /* Integral Anti-windup*/
        if(PID_Rotor.clamp_on && abs(PID_Pend.Set_point - PID_Pend.measurment) < 0.2*STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE)   //0.2
            PID_Rotor.int_term = lambda*PID_Rotor.int_term;
            //PID_Rotor.int_term = lambda*PID_Rotor.int_term - (1 - lambda)*PID_Rotor.int_term;
        /* Calculate Pendulum SP - PV*/
        *current_error_steps = ENCODER_ANGLE_POLARITY * (PID_Pend.Set_point - PID_Pend.measurment - PID_Rotor.control_output);

        pid_filter_control_execute(&PID_Pend, current_error_steps, contr_period);

        /* Convert PID Output to Angle*/
        rotor_control_target_steps = (PID_Pend.control_output)*Rotor_scale;
        (*task_Contr) = rotor_control_target_steps;
        //printf("Enc pos: %f\t Target steps: %f\tCurr Error steps: %f\n", encoder_position, rotor_control_target_steps, *current_error_steps);
    }
    else if (!balance_on) 
        (*task_Contr) = 0;
}
/*-----------------------------------------------------------*/

void vLetDummyTask_init(void) {
    /* Dummy Task for taking up CPU cycles*/
}
/*-----------------------------------------------------------*/

void vLetDummyTask_job(void) {

    uint32_t base_delay = 200000;   //200000

    uint32_t random = rand();
    uint32_t cycles = (random) % base_delay;//240000;

    BSP_WaitClkCycles(cycles);

}
/*-----------------------------------------------------------*/