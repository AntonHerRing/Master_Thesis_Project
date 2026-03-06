#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "bsp.h"

#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/structs/io_bank0.h"
//#include "hardware/pio.h"
//#include "hardware/"

#define ENCODER_SPR 2400


//Phase A and B GPIO ports for the Rotary Encoder
#define Phase_A 10
#define Phase_B 11

//TaskHandle_t    blinkTsk; /* Handle for the LED task. */
//TaskHandle_t    acclTsk; /* Handle for the accelerometer task. */
TaskHandle_t    encTsk; /* Handle for the rotary encoder task. */

/**
 * @brief Blink task.
 * 
 * @param args Task period (uint32_t).
 */
//void blink_task(void *args);
void enc_task(void *args);

//function definition
int grayTo_int(bool Enc_A, bool Enc_B);

void init_rotary_encoder(void);

//The Rotary (Gray code) Pulses
volatile bool Pulse_A = false;
volatile bool Pulse_B = false;

volatile int32_t count = 0;
volatile int dir = 0;

//volatile bool A_first = false;
//volatile bool B_first = false;


/*************************************************************/

/**
 * @brief Interrupt handler callback function.
 */
void gpio_callback(uint gpio, uint32_t events) {
    static bool A_first = false;
    static bool B_first = false;

    static int buffer = 0x00;
    if (events & GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL) {
        // Rising or falling edge detected
        if (gpio == Phase_A){ 
            //count++;
            dir = gpio_get(Phase_A) == gpio_get(Phase_B) ? 1 : -1;
            //count += dir >= 0 ? 1 : -1;
            count++;
        }
        if (gpio == Phase_B){
            //count++;
            dir = gpio_get(Phase_A) != gpio_get(Phase_B) ? 1 : -1;
            //count += dir >= 0 ? 1 : -1;
            count++;
        }

    }
}

/**
 * @brief Main function.
 * 
 * @return int 
 */
int main()
{
    BSP_Init();             /* Initialize all components on the lab-kit. */
    init_rotary_encoder();  /* Initialize the Rotary Encoder. */


    //Activate Interupt for 10 and 11
    gpio_set_irq_enabled_with_callback(Phase_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled(Phase_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    

    //gpio_set_irq_enabled_with_callback(Phase_A, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    //gpio_set_irq_enabled(Phase_B, GPIO_IRQ_EDGE_RISE, true);
    
    
    /* Create the tasks. */
    //xTaskCreate(blink_task, "Blink Task", 512, (void*) 1000, 2, &blinkTsk);
    xTaskCreate(enc_task, "Enc task", 512, (void*) 100, 2, &encTsk);

    
    vTaskStartScheduler();  /* Start the scheduler. */
    
    while (true) { 
        sleep_ms(1000); /* Should not reach here... */
    }
}
/*-----------------------------------------------------------*/


/*void enc_task(void *args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   // Get period (in ticks) from argument.

    int Full_rotation   = 360;        // Number of pulses for full rotation
    int curr_rot        = 0;

    int current_Pos = 0;
    int last_Pos    = 0;
    int diff_Pos    = 0;

    for (;;) {
        // GPIO 40 == Yellow == A, GPIO 39 == Green == B (WIP)

        //attatch to interrupt 
        current_Pos = grayTo_int(Pulse_A, Pulse_B);   //collect A and B encoder outputs. Convert to int
        //printf("Current Pos: %d\n", current_Pos);
        //printf("Last Pos: %d\n", last_Pos);

        printf("P_A: %d\tP_B: %d\tDiff: %d\tAng: %d\n",Pulse_A, Pulse_B, diff_Pos, curr_rot);

        diff_Pos = (last_Pos - current_Pos);                    //calc diff in pos
        //printf("Diff Pos: %d\n", diff_Pos);

        //turned one way
        if((diff_Pos == -1) || ( diff_Pos == 3)){               //determine directionbased on diff.
            last_Pos = current_Pos;
            curr_rot++;                                         //increase rotaional counter with enough pulses                                // adjust rotation to 0 - 360 Deg. 
        }
        //turned the other way
        else if((diff_Pos == 1) || ( diff_Pos == -3)){
            last_Pos = current_Pos;
            curr_rot--;
        }
        //Error: Missed input
        else if((diff_Pos == 2) || ( diff_Pos == -2)){}
            //printf("Error: Missed input\n");
        
        //printf("\n");
        //printf("Current angle: %d\n", curr_rot);
        //printf("Current pulses: %d\n", count);
        
        //last step in loop
        vTaskDelayUntil(&xLastWakeTime, xPeriod);   // Wait for the next release. 
    }   
}
/*-----------------------------------------------------------*/

void enc_task(void *args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   // Get period (in ticks) from argument.

    int local_count = 0;
    int local_dir   = 0;
    int diff        = 0;

    float last_deg    = 0;
    float deg       = 0.0;

    for (;;) {
        // GPIO 40 == Yellow == A, GPIO 39 == Green == B (WIP)
        printf("Count: %d\tDir: %d\n",count, dir);
        local_count = count;
        local_dir   = dir;
        

        local_count = local_count % ENCODER_SPR;
        local_count = local_count >= 0 ? local_count : local_count + ENCODER_SPR;
        deg = (float)local_count * (360.0 / ENCODER_SPR);

        diff = abs(deg - last_deg);
        deg = local_dir > 0 ? deg : last_deg - diff;
        
        printf("Count: %d\tDir: %d\tDeg: %f\tLast Deg: %f\n",count, dir, deg, last_deg);
        
        last_deg = deg;
        //last step in loop
        vTaskDelayUntil(&xLastWakeTime, xPeriod);   // Wait for the next release. 
    }   
}

int grayTo_int(bool Enc_A, bool Enc_B){
    if(!Enc_A && !Enc_B){
        return 0;
    }
    else if(!Enc_A && Enc_B){
        return 1;
    }
    else if(Enc_A && Enc_B){
        return 2;
    }
    else{
        return 3;
    }
}


void init_rotary_encoder(void){
    // initiate GPIOs
    gpio_init(Phase_A);
    gpio_set_dir(Phase_A, GPIO_IN);
    gpio_pull_up(Phase_A);

    gpio_init(Phase_B);
    gpio_set_dir(Phase_B, GPIO_IN);
    gpio_pull_up(Phase_B);

    // initiate interrupts in GPIO pins
    /*gpio_set_irq_enabled_with_callback(Phase_A, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled(Phase_B, GPIO_IRQ_EDGE_RISE, true);

    //initiate PIO
    PIO pio = pio0;
    
    pio_gpio_init(pio, Phase_A);
    pio_gpio_init(pio, Phase_B);
    
    //load encoder program into PIO Memory
    uint offset = pio_add_program(pio, &pio_rot_enc_program);*/
    
}
