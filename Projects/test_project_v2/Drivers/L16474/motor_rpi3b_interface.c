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

extern void L6474_StepClockHandler(uint8_t deviceId);

void flag_pin_isr(void)
{
    printf("flag pin interruption! \n");
}

void pwm_pin_isr(void)
{
    L6474_StepClockHandler(0);
}

/******************************************************//**
 * @brief This function provides an accurate delay in milliseconds
 * @param[in] delay  time length in milliseconds
 * @retval None
 **********************************************************/
void L6474_Board_Delay(uint32_t milliseconds)
{
	//delay(milliseconds);
    sleep_ms(milliseconds);
}

/******************************************************//**
 * @brief  Initiliases the GPIOs used by the L6474s
 * @retval None
  **********************************************************/
void L6474_Board_GpioInit() {
	/* Configure L6474 - Flag pin -------------------------------------------*/
    gpio_init(FLAG_PIN);
    gpio_set_function(FLAG_PIN, GPIO_IN);
    gpio_pull_up(FLAG_PIN);
    //pinMode(FLAG_PIN, INPUT);
    //pullUpDnControl(FLAG_PIN, PUD_UP);

    /*if (wiringPiISR(FLAG_PIN, INT_EDGE_FALLING, &flag_pin_isr) < 0) {
        perror("wiringPiISR");
        exit(EXIT_FAILURE);
    }*/
   gpio_set_irq_enabled(FLAG_PIN, GPIO_IRQ_EDGE_FALL, true);

	/* Configure L6474 - STBY/RESET pin -------------------------------------*/
    //pinMode(RESET_PIN, OUTPUT);
    gpio_init(RESET_PIN);
    gpio_set_function(RESET_PIN, GPIO_OUT);
    L6474_Board_Reset();

    /* Configure L6474 - DIR pin for first device  -------------------------------*/
    //pinMode(DIR_PIN, OUTPUT);
    gpio_init(DIR_PIN);
    gpio_set_function(DIR_PIN, GPIO_OUT);
}

/******************************************************//**
 * @brief  Sets the frequency of PWM
 * @param[in] newFreq in Hz
 * @retval None
 * @note The frequency is directly the current speed of the device
 **********************************************************/
void L6474_Board_PwmSetFreq(uint16_t newFreq)
{
    int intensity = 0.5 * PWM_range;
    uint16_t divisor;
    divisor = SYSFREQ / PWM_range / newFreq;
    //pwmSetClock(divisor);
    pwm_set_clkdiv(pwm_gpio_to_slice_num(PWM_PIN), divisor);
    pwm_set_gpio_level(PWM_PIN, intensity);


}


/******************************************************//**
 * @brief  Initialises the PWM uses by the specified device
 * @retval None
 **********************************************************/
void L6474_Board_PwmInit()
{
    //pinMode(PWM_PIN, PWM_OUTPUT);
    //pwmSetMode(PWM_MODE_MS);
    //pwmSetRange(PWM_range);
    //pwmWrite(PWM_PIN, 0);
    gpio_init(PWM_PIN);
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_wrap(slice_num, PWM_range);
    pwm_set_gpio_level(PWM_PIN, 0);
    pwm_set_enabled(slice_num, true);
    

    //pinMode(PWM_TIMER_PIN, INPUT);
    //pullUpDnControl(PWM_TIMER_PIN, PUD_UP);

    gpio_init(PWM_TIMER_PIN);
    gpio_set_function(PWM_TIMER_PIN, GPIO_IN);
    gpio_pull_up(PWM_TIMER_PIN);
    gpio_set_irq_enabled(PWM_TIMER_PIN, GPIO_IRQ_EDGE_RISE, true);

    /*if (wiringPiISR(PWM_TIMER_PIN, INT_EDGE_RISING, &pwm_pin_isr) < 0) {
        perror("wiringPiISR");
        exit(EXIT_FAILURE);
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

    spi_init(SPI_PORT, 1 * 1000 * 1000); // 1 * 1000 * 1000 = 1MHz

    gpio_set_function(SPI_CS, GPIO_FUNC_SPI);       /* CS */
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
void L6474_Board_SpiWriteBytes(uint8_t* pByteToTransmit, uint8_t len)
{
    //uint8_t fd;
    //fd = wiringPiSPIDataRW(SPI_CHANNEL, pByteToTransmit, 1);

    spi_write_blocking (SPI_PORT, pByteToTransmit, len);
    gpio_put(SPI_SCK, true);
    gpio_put(SPI_SCK, false);
}
