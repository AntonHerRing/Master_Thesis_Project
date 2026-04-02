#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "trace.h"
#include "let.h"
#include "bsp.h"

TaskHandle_t    blinkTsk; /* Handle for the LED task. */

/**
 * @brief Blink task.
 * 
 * @param args Task period (uint32_t).
 */
void blink_task(void *args);

label_t label_A;    /* Data label used for LET tasks. */

LetTask_t letTask1; /* Handle for LET task 1. */
LetTask_t letTask2; /* Handle for LET task 2. */

uint32_t* task1_A;      /* Pointer to the local data of label A by task 1. */
uint32_t  task1_A_data; /* Local copy of label A owned by LET task 1. */
uint32_t* task2_A;      /* Pointer to the local data of label A by task 2. */
uint32_t  task2_A_data; /* Local copy of label A owned by LET task 2. */

/**
 * @brief Initialization function of LET task 1.
 */
void vLetTask1_init(void);

/**
 * @brief Job function of LET task 1.
 */
void vLetTask1_job(void);

/**
 * @brief Initialization function of LET task 2.
 */
void vLetTask2_init(void);

/**
 * @brief Job function of LET task 2.
 */
void vLetTask2_job(void);


/*************************************************************/

/**
 * @brief Main function.
 * 
 * @return int 
 */
int main()
{
    BSP_Init();             /* Initialize all components on the lab-kit. */
    trace_init();           /* Initialize the traceing infrastructure. */
    
    if (xLetInit() == pdFALSE) {                    /* Initialize the LET module. */
        while (true);
    }

    /* Create the tasks. */
    xTaskCreate(blink_task, "Blink Task", 512, (void*) 1000, 2, &blinkTsk);

    /* Create a label and LET tasks that read/write from it. */
    xLetInitLabel("A", sizeof(uint32_t), &label_A, LET_COM_COPY);

    xLetTaskCreate(vLetTask1_init, vLetTask1_job, "LET_Task_1", 512, 5, 500, 500, 0, CORE0, &letTask1);
    xLetTaskCreate(vLetTask2_init, vLetTask2_job, "LET_Task_2", 512, 5, 500, 500, 0, CORE0, &letTask2);
    
    vTaskStartScheduler();  /* Start the scheduler. */
    
    while (true) { 
        sleep_ms(1000); /* Should not reach here... */
    }
}
/*-----------------------------------------------------------*/

void blink_task(void *args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   /* Get period (in ticks) from argument. */

    for (;;) {
        BSP_ToggleLED(LED_GREEN);

        vTaskDelayUntil(&xLastWakeTime, xPeriod);   /* Wait for the next release. */
    }
}
/*-----------------------------------------------------------*/

void vLetTask1_init(void) {
    task1_A = &task1_A_data;    /* Initialize the pointer to the local buffer for label A */
    xLetTaskRegisterWrite(&letTask1, &label_A, (void*) &task1_A);    /* Register the write access for label A */    
}
/*-----------------------------------------------------------*/

void vLetTask1_job(void) {
  static uint32_t input = 0;

  input++;

    (*task1_A) = input;
}
/*-----------------------------------------------------------*/

void vLetTask2_init(void) {
    task2_A = &task2_A_data;    /* Initialize the pointer to the local buffer for label A */
    xLetTaskRegisterRead(&letTask2, &label_A, (void*) &task2_A);    /* Register the read access for label A */    
}
/*-----------------------------------------------------------*/

void vLetTask2_job(void) {
    printf("%u\r\n", *task2_A);
}
/*-----------------------------------------------------------*/
