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

