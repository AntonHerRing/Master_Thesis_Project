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

#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/structs/io_bank0.h"

#include "../project_LET/Drivers/L16474/motor_rpi3b_interface.h"
#include "../project_LET/Drivers/L16474/l6474.h"
#include "../project_LET/Drivers/L16474/steppermotor.h"

/*#include "Control/Control.h"
#include "Encoder/Encoder.h"*/

#include "../project_LET/Tasks/tasks.h"

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



TaskHandle_t EncTask;
TaskHandle_t ContrTask;
TaskHandle_t BtnsTask;
TaskHandle_t MotorTask;
TaskHandle_t PrintTask;
TaskHandle_t DummyTask;

void Enc_Task(void *args);

void Contr_Task(void *args);

void Btns_Task(void *args);

void Motor_Task(void *args);

void Print_Task(void *args);

void Dummy_Task(void *args);

/** Make Dummy functions to LET functions **/
UBaseType_t xLetTaskRegisterWrite(LetTask_t *pxLetTask, label_t *pxLabel, void **pxLocalBufferPtr){
    
}

UBaseType_t xLetTaskRegisterRead(LetTask_t *pxLetTask, label_t *pxLabel, void **pxLocalBufferPtr){

}


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
    
    xTaskCreate(Enc_Task, "Enc Task", 512, (void*) T_Enc, 7, &EncTask);
    vTaskCoreAffinitySet(EncTask, CORE1);

    xTaskCreate(Motor_Task, "Motor Task", 18216, (void*) T_Motor, 6, &MotorTask);
    vTaskCoreAffinitySet(MotorTask, CORE0);

    xTaskCreate(Btns_Task, "Btns Task", 512, (void*) T_Btns, 5, &BtnsTask);
    vTaskCoreAffinitySet(BtnsTask, CORE1);

    /* Dummy Task for taking up space on Scheduler*/
    //xTaskCreate(Dummy_Task, "Dummy Task", 5120, (void*) T_Dummy, 4, &DummyTask);
    //vTaskCoreAffinitySet(DummyTask, CORE0);

    xTaskCreate(Contr_Task, "Contr Task", 5120, (void*) T_Contr, 3, &ContrTask);
    vTaskCoreAffinitySet(ContrTask, CORE0);

    xTaskCreate(Print_Task, "Print Task", 1024, (void*) T_Print, 2, &PrintTask);
    vTaskCoreAffinitySet(PrintTask, CORE0);

   
    vTaskStartScheduler();  /* Start the scheduler. */
    
    while (true) { 
        sleep_ms(1000); /* Should not reach here... */
    }
}

/*-----------------------------------------------------------*/

void Btns_Task(void *args) {
    TickType_t xLastWakeTime = 5;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */
    uint8_t buttons = 0;

    vLetBtnsTask_init();

    vTaskDelayUntil(&xLastWakeTime, 0);

    for (;;) {
        uint8_t btn1 = BSP_GetInput(SW_5);
        uint8_t btn2 = BSP_GetInput(SW_6);
        uint8_t btn3 = BSP_GetInput(SW_7);
        uint8_t btn4 = BSP_GetInput(SW_8);

        buttons = 0x0 | (!btn1 | (!btn2 << 1) | (!btn3 << 2) | (!btn4 << 3));

        taskENTER_CRITICAL();
        (*task_Btns) = buttons;
        taskEXIT_CRITICAL();

        vTaskDelayUntil(&xLastWakeTime, xPeriod);   /* Wait for the next release. */
    }
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

void Enc_Task(void *args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */

    float encoder_value = 0;
    uint8_t button = 0;

    vLetEncTask_init();

    vTaskDelayUntil(&xLastWakeTime, ENC_OFFSET);

    for (;;) {   
        //Handle button inputs
        taskENTER_CRITICAL();
        button = *task_Btns;
        taskEXIT_CRITICAL();

        switch(button){
            case 4: count = 0; break; //reset pendulum angle
            default: break;
        }

        encoder_value = get_encoder_angle_continous(count);
        taskENTER_CRITICAL();
        (*task_Enc) = encoder_value;
        taskEXIT_CRITICAL();

        vTaskDelayUntil(&xLastWakeTime, xPeriod);   /* Wait for the next release. */
    }
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

void Motor_Task(void *args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */

    vLetMotorTask_init();

    /******** Init static var ********/
    float motor_deg = 0.0;
    float desired_pos = 0.0;

    int8_t buttons = 0;
    float motor_read = 0;
    bool pos_overflow = false;
    float collector = 0;

    vTaskDelayUntil(&xLastWakeTime, 0);

    for (;;) {
        /******** Main function ********/
        //Handle button inputs
        taskENTER_CRITICAL();
        buttons = *task_Btns;
        motor_read = *task_Contr;
        taskEXIT_CRITICAL();

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
        desired_pos = motor_read - collector;
        motor_deg = get_stepper_angle();
        
        // Catch Control signal overflow
        if(!pos_overflow && abs(motor_deg) >= 360 || abs(desired_pos) >= 360){
            L6474_HardStop(0);
            pos_overflow = true;
        }

        // Move if signal is stable
        if(!pos_overflow){
            move_stepper_to(desired_pos);
        }
        taskENTER_CRITICAL();
        (*task_Motor) = motor_deg; //write any inputs
        taskEXIT_CRITICAL();

        //printf("Motor: %f\n", motor_deg);

        vTaskDelayUntil(&xLastWakeTime, xPeriod);   /* Wait for the next release. */
    }
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

void Print_Task(void *args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */

    vLetPrintTask_init();

    /******* Init static var *******/
    uint32_t run_time = 0; 
    float Encoder = 0;
    float Motor = 0;
    float Controller = 0;
    
    float inc_mean       = 0;
    float M2             = 0;
    float past_inc_mean  = 0;

    float inc_variance   = 0;
    float inc_stndDev    = 0;
    float samples        = 0;

    bool activate_calc = CALC_ON;
    bool set_point_reached = false;

    vTaskDelayUntil(&xLastWakeTime, 0);

    for (;;) {
        run_time += T_Print;
        taskENTER_CRITICAL();
        Encoder = *task_Enc;
        Motor = *task_Motor;
        Controller = *task_Contr;
        taskEXIT_CRITICAL();

        //print data
        printf("#-42-#: Run Time(s): %f\tDeg: %f\tMotor Deg: %f\tTarget Deg: %f\tEnd\r\n", 
                (float)run_time/1000.0f, Encoder, Motor, Controller); //Read any inputs

        if(!set_point_reached && (int)Encoder == 180 ){
            set_point_reached = true;
        }

        /* Calculate Incremental mean, standard deviation and Variance*/
        if (activate_calc && set_point_reached){
            samples++;
            past_inc_mean = inc_mean;
            inc_mean = inc_mean + (Encoder - inc_mean)/samples;

            M2 = M2 + (Encoder - past_inc_mean)*(Encoder - inc_mean);

            if(samples > 2){
                /* Sample Variance */
                /* Welfrod Variance */
                inc_variance = M2 / (samples - 1.0f);
            }

            inc_stndDev = sqrt(inc_variance);
            printf("##32##: Samples: %f\tMean: %f\tVariance: %f\tStandard Deviation: %f\tEnd\r\n", samples, inc_mean, inc_variance, inc_stndDev);
        }
        
        vTaskDelayUntil(&xLastWakeTime, xPeriod);   /* Wait for the next release. */
    }
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

void Contr_Task(void *args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */

    vLetContrTask_init();

    float lambda = 0.91;
    float Encoder_read = 0;
    float Motor_read = 0;
    float Controll_write = 0;

    uint32_t current_time = 0;
    uint32_t start_time = 0;
    bool sp_changed = false;

    vTaskDelayUntil(&xLastWakeTime, 0);

    for (;;) {
        taskENTER_CRITICAL();
        Encoder_read = *task_Enc;
        Motor_read = *task_Motor;
        taskEXIT_CRITICAL();
        
        /* Activate Balancing*/
        if (abs(Encoder_read) >= 179.5 && abs(Encoder_read) <= 180.5 && balance_on == false){
            balance_on = true;
            L6474_SetAnalogValue(0, L6474_TVAL, MAX_TORQUE_CONFIG);
        }

        /* Activation for step response */
        if (STEP_RESPONSE && (current_time - start_time) >= 10000){
            PID_Rotor.Set_point = 15 * STEPPER_READ_POSITION_STEPS_PER_DEGREE;
            //PID_Rotor.Set_point = 3.324262676 * STEPPER_READ_POSITION_STEPS_PER_DEGREE;
            //printf("--------------Sp changed--------------\n");
        }
    
        if (balance_on && (Encoder_read > 150 &&  Encoder_read < 210)){
            /* Activation for step response */
            if(STEP_RESPONSE && !sp_changed){
                start_time = xTaskGetTickCount();
                sp_changed = true;
            }
            if (STEP_RESPONSE) current_time = xTaskGetTickCount();

            /* Calculate Rotor SP - PV*/
            PID_Rotor.measurment = Motor_read * STEPPER_READ_POSITION_STEPS_PER_DEGREE;
            *current_error_rotor_steps = PID_Rotor.Set_point - PID_Rotor.measurment;

            pid_filter_control_execute(&PID_Rotor, current_error_rotor_steps, contr_period);

            PID_Pend.measurment = Encoder_read * STEPPER_READ_POSITION_STEPS_PER_DEGREE;
            /* Integral Anti-windup*/
            if(PID_Rotor.clamp_on && abs(PID_Pend.Set_point - PID_Pend.measurment) < 0.2*STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE)   //0.2
                PID_Rotor.int_term = lambda*PID_Rotor.int_term - (1 - lambda)*PID_Rotor.int_term;
            /* Calculate Pendulum SP - PV*/
            *current_error_steps = ENCODER_ANGLE_POLARITY * (PID_Pend.Set_point - PID_Pend.measurment - PID_Rotor.control_output);

            pid_filter_control_execute(&PID_Pend, current_error_steps, contr_period);

            /* Convert PID Output to Angle*/
            rotor_control_target_steps = (PID_Pend.control_output)*Rotor_scale;
            Controll_write = rotor_control_target_steps;
            //printf("Enc pos: %f\t Target steps: %f\tCurr Error steps: %f\n", encoder_position, rotor_control_target_steps, *current_error_steps);
        }
        else if (!balance_on) 
            Controll_write = 0;
            
        taskENTER_CRITICAL();    
        (*task_Contr) = Controll_write;
        taskEXIT_CRITICAL();

        vTaskDelayUntil(&xLastWakeTime, xPeriod);   /* Wait for the next release. */
    }
}
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/

void Dummy_Task(void *args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */

    vLetDummyTask_init();

    vTaskDelayUntil(&xLastWakeTime, 0);

    for (;;) {

        //BSP_WaitClkCycles(270000);
        vLetDummyTask_job();
        
        vTaskDelayUntil(&xLastWakeTime, xPeriod);   /* Wait for the next release. */
    }
}
/*-----------------------------------------------------------*/
