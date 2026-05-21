//#pragma GCC optimize ("O0") /* Incldue for dubuggning. Easier viewing of variables */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "bsp.h"
//#include "let.h"
#include "trace.h"

#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/structs/io_bank0.h"

#include "Drivers/L16474/motor_rpi3b_interface.h"
#include "Drivers/L16474/l6474.h"
#include "Drivers/L16474/steppermotor.h"

/*#include "Control/Control.h"
#include "Encoder/Encoder.h"*/

#include "Tasks/tasks.h"

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

void Enc_Task(void *args);

void Contr_Task(void *args);

void Btns_Task(void *args);

void Motor_Task(void *args);

void Print_Task(void *args);

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
    
    //xTaskCreate(Enc_Task, "Enc Task", 512, (void*) T_Enc, 6, &EncTask);
    xTaskCreate(Contr_Task, "Contr Task", 5120, (void*) T_Contr, 5, &ContrTask);
    xTaskCreate(Btns_Task, "Btns Task", 512, (void*) T_Btns, 4, &BtnsTask);
    xTaskCreate(Motor_Task, "Motor Task", 18216, (void*) T_Motor, 3, &MotorTask);
    xTaskCreate(Print_Task, "Print Task", 1024, (void*) T_Print, 2, &PrintTask);

    //low num = low prio, High num = high prio
    /*xLetTaskCreate(vLetEncTask_init, vLetEncTask_job, "LET_Enc_Task", 512, 6, T_Enc, T_Enc, 0, CORE1, &letEncTsk);
    xLetTaskCreate(vLetContrTask_init, vLetContrTask_job, "LET_Control_Task", 5120, 5, T_Contr, T_Contr, 0, CORE0, &letContrTsk);
    xLetTaskCreate(vLetBtnsTask_init, vLetBtnsTask_job, "LET_Buttons_Task", 512, 4, T_Btns, T_Btns, 0, CORE0, &letBtnsTsk);
    xLetTaskCreate(vLetMotorTask_init, vLetMotorTask_job, "LET_Motor_Task", 18216, 3, T_Motor, T_Motor, 0, CORE0, &letMotorTsk);    //18216          //10240, is too much
    xLetTaskCreate(vLetPrintTask_init, vLetPrintTask_job, "LET_Print_Task", 1024, 2, T_Print, T_Print, 0, CORE0, &letPrintTsk);*/
    
    vTaskStartScheduler();  /* Start the scheduler. */
    
    while (true) { 
        sleep_ms(1000); /* Should not reach here... */
    }
}

/*-----------------------------------------------------------*/

void Btns_Task(void *args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */

    vLetBtnsTask_init();

    for (;;) {
        uint8_t btn1 = BSP_GetInput(SW_5);
        uint8_t btn2 = BSP_GetInput(SW_6);
        uint8_t btn3 = BSP_GetInput(SW_7);
        uint8_t btn4 = BSP_GetInput(SW_8);

        uint8_t buttons = 0x0 | (!btn1 | (!btn2 << 1) | (!btn3 << 2) | (!btn4 << 3));

        taskENTER_CRITICAL();
        *task_Btns = buttons;
        taskEXIT_CRITICAL();

        //printf("Buttons: %d\n", buttons);

        vTaskDelayUntil(&xLastWakeTime, xPeriod);   /* Wait for the next release. */
    }
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

void Enc_Task(void *args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */

    float encoder_value = 0;

    vLetEncTask_init();

    for (;;) {   
        //Handle button inputs
        taskENTER_CRITICAL();
        uint8_t button = *task_Btns;
        taskEXIT_CRITICAL();

        switch(button){
            case 4: count = 0; break; //reset pendulum angle
            default: break;
        }

        encoder_value = get_encoder_angle_continous(count);
        taskENTER_CRITICAL();
        (*task_Enc) = encoder_value;
        taskEXIT_CRITICAL();

        //printf("Encoder: %f\n", encoder_value);

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
    
        if (balance_on && (Encoder_read > 150 &&  Encoder_read < 210)){
            /* Calculate Rotor SP - PV*/
            PID_Rotor.measurment = Motor_read * STEPPER_READ_POSITION_STEPS_PER_DEGREE;
            *current_error_rotor_steps = PID_Rotor.Set_point - PID_Rotor.measurment;

            pid_filter_control_executeV3(&PID_Rotor, current_error_rotor_steps, contr_period);

            PID_Pend.measurment = Encoder_read * STEPPER_READ_POSITION_STEPS_PER_DEGREE;
            /* Integral Anti-windup*/
            if(PID_Rotor.clamp_on && abs(PID_Pend.Set_point - PID_Pend.measurment) < 0.2*STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE)   //0.2
                PID_Rotor.int_term = lambda*PID_Rotor.int_term - (1 - lambda)*PID_Rotor.int_term;
            /* Calculate Pendulum SP - PV*/
            *current_error_steps = ENCODER_ANGLE_POLARITY * (PID_Pend.Set_point - PID_Pend.measurment - PID_Rotor.control_output);

            pid_filter_control_executeV3(&PID_Pend, current_error_steps, contr_period);

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

        //printf("Controller: %f\n", Controll_write);

        vTaskDelayUntil(&xLastWakeTime, xPeriod);   /* Wait for the next release. */
    }
}
/*-----------------------------------------------------------*/

