//#pragma GCC optimize ("O0") /* Incldue for dubuggning. Easier viewing of variables */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
//#include <wiringPi.h>
//#include <wiringPiSPI.h>
#include "motor_rpi3b_interface.h"

#include "bsp.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "psram.h"
#include "FreeRTOS.h"

#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/structs/io_bank0.h"

#define Phase_A 40
#define Phase_B 39

extern void L6474_StepClockHandler(uint8_t deviceId);

struct repeating_timer timer;

volatile uint32_t timer_delay_us;

void flag_pin_isr(void)
{
    printf("flag pin interruption! \n");
}

//void pwm_pin_isr(void)
/*bool pwm_pin_isr(struct repeating_timer *t)
{
    L6474_StepClockHandler(0);
    return true;

}*/

// Handler for PWM IRQ
void pwm_pin_isr(void){
    uint slice = pwm_gpio_to_slice_num(PWM_PIN);

    if (pwm_get_irq_status_mask() & (1u << slice)) {
        pwm_clear_irq(slice);

        L6474_StepClockHandler(0);
    }
}

/******************************************************//**
 * @brief This function provides an accurate delay in milliseconds
 * @param[in] delay  time length in milliseconds
 * @retval None
 **********************************************************/
void L6474_Board_Delay(uint32_t milliseconds)
{
    sleep_ms(milliseconds);
    //vTaskDelay(milliseconds);
}

/******************************************************//**
 * @brief  Initiliases the GPIOs used by the L6474s
 * @retval None
  **********************************************************/
void L6474_Board_GpioInit() {
	/* Configure L6474 - Flag pin -------------------------------------------*/
    gpio_init(FLAG_PIN);
    gpio_set_dir(FLAG_PIN, GPIO_IN);
    gpio_pull_up(FLAG_PIN);
    //pinMode(FLAG_PIN, INPUT);
    //pullUpDnControl(FLAG_PIN, PUD_UP);

    /*if (wiringPiISR(FLAG_PIN, INT_EDGE_FALLING, &flag_pin_isr) < 0) {
        perror("wiringPiISR");
        exit(EXIT_FAILURE);
    }*/
    gpio_set_irq_enabled(FLAG_PIN, GPIO_IRQ_EDGE_FALL, true);
    //irq_set_exclusive_handler(FLAG_PIN, &flag_pin_isr);
    //irq_set_enabled(FLAG_PIN, true);


	/* Configure L6474 - STBY/RESET pin -------------------------------------*/
    //pinMode(RESET_PIN, OUTPUT);
    gpio_init(RESET_PIN);
    gpio_set_dir(RESET_PIN, GPIO_OUT);
    L6474_Board_Reset();

    /* Configure L6474 - DIR pin for first device  -------------------------------*/
    //pinMode(DIR_PIN, OUTPUT);
    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
}

/******************************************************//**
 * @brief  Sets the frequency of PWM
 * @param[in] newFreq in Hz
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void L6474_Board_PwmSetFreq(uint16_t newFreq)
{
    uint slice = pwm_gpio_to_slice_num(PWM_PIN);

    float divisor = (float)SYSFREQ / (PWM_range * newFreq);
    pwm_set_clkdiv(slice, divisor);

    pwm_set_gpio_level(PWM_PIN, 0.5 * PWM_range); // 50% duty
}


/******************************************************//**
 * @brief  Initialises the PWM uses by the specified device
 * @retval None
 **********************************************************/
void L6474_Board_PwmInit()
{
    int dummy = 0;

    timer_delay_us = 20000; // default 20kHz

    gpio_init(PWM_PIN);
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    gpio_pull_up(PWM_PIN);
    
    uint slice = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_wrap(slice, PWM_range);

    //float divisor = (float)SYSFREQ / ((PWM_range + 1) * 20000); // default 20kHz
    float divisor = (float)SYSFREQ / (PWM_range * timer_delay_us); // default 20kHz
    pwm_set_clkdiv(slice, divisor);

    pwm_set_gpio_level(PWM_PIN, 0.5 * PWM_range);
    pwm_set_enabled(slice, true);

    // Initiate PWM IRQ and trigger-callback 
    pwm_clear_irq(slice);             
    pwm_set_irq_enabled(slice, true); 
    irq_set_exclusive_handler(PWM_IRQ_WRAP, &pwm_pin_isr);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // 1M * 1/f => us
    /*if (add_repeating_timer_us(1000000 / timer_delay_us, &pwm_pin_isr, NULL, &timer) == false){
        dummy = 0;
    }*/
    
}

/******************************************************//**
 * @brief  Stops the PWM uses by the specified device
 * @retval None
 **********************************************************/
void L6474_Board_PwmStop()
{
    pwm_set_gpio_level(PWM_PIN, 0);
}

/******************************************************//**
 * @brief  Releases the L6474 reset (pin set to High) of all devices
 * @retval None
 **********************************************************/
void L6474_Board_ReleaseReset()
{
    gpio_put(RESET_PIN, true);
}

/******************************************************//**
 * @brief  Resets the L6474 (reset pin set to low) of all devices
 * @retval None
 **********************************************************/
void L6474_Board_Reset()
{
    gpio_put(RESET_PIN, false);
}

/******************************************************//**
 * @brief  Set the GPIO used for the direction
 * @param[in] gpioState state of the direction gpio (0 to reset, 1 to set)
 * @retval None
 **********************************************************/
void L6474_Board_SetDirectionGpio(uint8_t gpioState)
{
    switch (gpioState)
    {
        case 1:
            //digitalWrite(DIR_PIN, HIGH);
            gpio_put(DIR_PIN, true);
            break;
        case 0: 
            //digitalWrite(DIR_PIN, LOW);
            gpio_put(DIR_PIN, false);
            break;
        default:
            ;
    }
}

/******************************************************//**
 * @brief  Initialise the SPI used by L6474
 * @retval return file descriptor if SPI transaction is OK, -1 else
 **********************************************************/
void L6474_Board_SpiInit()
{
    //fd = wiringPiSPISetupMode(SPI_CHANNEL, 500000, 3);

    gpio_init(SPI_CS);
    gpio_init(SPI_SCK);
    gpio_init(SPI_MOSI);
    gpio_init(SPI_MISO);

    //spi_init(SPI_PORT, 5 * 100 * 1000); // 5 * 100 * 1000 = 500kHz
    spi_init(SPI_PORT, 5 * 100 * 1000); // 5 * 100 * 1000 = 500kHz
    spi_set_format(SPI_PORT, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    gpio_set_dir(SPI_CS, GPIO_OUT);
    gpio_put(SPI_CS, true);       /* CS */
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);      /* CLK */
    gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);     /* MOSI */
    gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);     /* MISO */
}

/******************************************************//**
 * @brief  Write and read SPI byte to the L6474
 * @param[in] pByteToTransmit pointer to the byte to transmit
 * @param[in] pReceivedByte pointer to the received byte
 * @retval HAL_OK if SPI transaction is OK, HAL_KO else
 **********************************************************/
uint8_t L6474_Board_SpiWriteBytes(uint8_t* pByteToTransmit, uint8_t* pReceivedByte, uint8_t nbDevices)
{
    //uint8_t fd;
    //fd = wiringPiSPIDataRW(SPI_CHANNEL, pByteToTransmit, 1);
    gpio_put(SPI_CS, false);
    //spi_write_blocking (SPI_PORT, pByteToTransmit, len);
    spi_write_read_blocking(SPI_PORT, pByteToTransmit, pReceivedByte, 1);

    gpio_put(SPI_CS, true);
    //gpio_put(SPI_SCK, true);
    //gpio_put(SPI_SCK, false);

    return *pReceivedByte;
}

uint8_t L6474_ReadByte(uint8_t* pByteToTransmit, uint8_t* pReceivedByte)
{
    spi_write_read_blocking(SPI_PORT, pByteToTransmit, pReceivedByte, 1);
    return *pReceivedByte;
}

void L6474_Board_EnableIrq(){
    irq_set_enabled(PWM_IRQ_WRAP, true);
    //portEXIT_CRITICAL();
}


void L6474_Board_DisableIrq(){
    irq_set_enabled(PWM_IRQ_WRAP, false);
    //portEXIT_CRITICAL();
}
