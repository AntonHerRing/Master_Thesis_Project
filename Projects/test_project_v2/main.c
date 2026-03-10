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

#include "Motor.h"
#include "Drivers/L16474/motor_rpi3b_interface.h"
#include "Drivers/L16474/l6474.h"
#include "Drivers/L16474/steppermotor.h"

#define ENCODER_SPR 2400

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


//Phase A and B GPIO ports for the Rotary Encoder
#define Phase_A 40
#define Phase_B 39



//TaskHandle_t    blinkTsk; /* Handle for the LED task. */
//TaskHandle_t    acclTsk; /* Handle for the accelerometer task. */
TaskHandle_t    encTsk; /* Handle for the rotary encoder task. */
TaskHandle_t    motorTsk; /* Handle for the rotary encoder task. */

/**
 * @brief Blink task.
 * 
 * @param args Task period (uint32_t).
 */
//void blink_task(void *args);
void enc_task(void *args);
void motor_task(void *args);

//function definition
int grayTo_int(bool Enc_A, bool Enc_B);

void init_rotary_encoder(void);

extern void L6474_StepClockHandler(uint8_t deviceId);

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
            dir = gpio_get(Phase_A) == gpio_get(Phase_B) ? 1 : -1;
            count += dir >= 0 ? 1 : -1;
            //count++;
        }
        if (gpio == Phase_B){
            dir = gpio_get(Phase_A) != gpio_get(Phase_B) ? 1 : -1;
            count += dir >= 0 ? 1 : -1;
            //count++;
        }

        if(gpio == PWM_PIN){
            L6474_StepClockHandler(0);
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

    init_motor();

    //Activate Interupt for 10 and 11
    gpio_set_irq_enabled_with_callback(Phase_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled(Phase_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    //motor pins
    //spi_init(SPI_PORT, 1 * 1000 * 1000); // 1 * 1000 * 1000 = 1MHz
    //gpio_set_function(SPI_CS, GPIO_FUNC_SPI);       /* CS */
    //gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);      /* CLK */
    //gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);     /* MOSI */
    //gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);     /* MISO */

    /* Create the tasks. */
    //xTaskCreate(blink_task, "Blink Task", 512, (void*) 1000, 2, &blinkTsk);
    xTaskCreate(enc_task, "Enc task", 512, (void*) 100, 2, &encTsk);
    xTaskCreate(motor_task, "Motor task", 512, (void*) 2000, 2, &motorTsk);

    
    vTaskStartScheduler();  /* Start the scheduler. */
    
    while (true) { 
        sleep_ms(1000); /* Should not reach here... */
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
        //printf("Count: %d\tDir: %d\n",count, dir);
        local_count = count;
        local_dir   = dir;
        

        local_count = local_count % ENCODER_SPR;
        local_count = local_count >= 0 ? local_count : local_count + ENCODER_SPR;
        deg = (float)local_count * (360.0 / ENCODER_SPR);

        //diff = abs(deg - last_deg);
        //deg = local_dir > 0 ? deg : last_deg - diff;
        
        //printf("Count: %d\tDir: %d\tDeg: %f\tLast Deg: %f\n",count, dir, deg, last_deg);
        printf("Deg: %f\n", deg);
        
        last_deg = deg;
        //last step in loop
        vTaskDelayUntil(&xLastWakeTime, xPeriod);   // Wait for the next release. 
    }   
}

void motor_task(void *args) {
    TickType_t xLastWakeTime = 0;
    const TickType_t xPeriod = (int)args;   // Get period (in ticks) from argument.

    int max_pos = 50;
    int curr_pos = 25;
    int min_pos = 0;

    int dir = 1;

    float motor_deg = 0.0;

    bool toggle = true;

    for (;;) {

        //motor_deg = get_stepper_angle();

        if(toggle){
            move_stepper_by(1.0);
            toggle = false;
        }

        //printf("Motor deg: %f\n", motor_deg);
     
        vTaskDelayUntil(&xLastWakeTime, xPeriod);   // Wait for the next release. 
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
    
}

/******* RED_alt *******/

