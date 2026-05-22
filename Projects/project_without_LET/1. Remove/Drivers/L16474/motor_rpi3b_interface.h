//translated to ES-Lab-board
#define FLAG_PIN 2
#define RESET_PIN 3
#define PWM_PIN 45 //PWN1 //23
#define DIR_PIN 41 
#define PWM_TIMER_PIN 44 //on pin laneled PWN2 (but not func PWN)//22 

#define SPI_CS 9
#define SPI_SCK 10
#define SPI_MOSI 11
#define SPI_MISO 12

#define SYSFREQ 150000000
#define SPI_CHANNEL 0
//static const int PWM_range = 1024;
#define PWM_range 1024

// void (*flag_pin_isr)(void);

/** @defgroup RPI3B_Board_Private_Function_Prototypes IHM01A1 Board Private Function Prototypes
  * @{
  */
void L6474_Board_Delay(uint32_t milliseconds);         //Delay of the requested number of milliseconds
void L6474_Board_GpioInit();   //Initialise GPIOs used for L6474s
void L6474_Board_PwmSetFreq(uint16_t newFreq); //Set PWM frequency and start it
void L6474_Board_PwmInit();    //Init the PWM of the specified device
void L6474_Board_PwmStop();    //Stop the PWM of the specified device
void L6474_Board_ReleaseReset(); //Reset the L6474 reset pin 
void L6474_Board_Reset();       //Set the L6474 reset pin 
void L6474_Board_SetDirectionGpio(uint8_t gpioState); //Set direction GPIO
void L6474_Board_SpiInit();   //Initialise the SPI used for L6474s
uint8_t L6474_Board_SpiWriteBytes(uint8_t* pByteToTransmit, uint8_t* pReceivedByte, uint8_t nbDevices); //Write bytes to the L6474s via SPI

uint8_t L6474_ReadByte(uint8_t* pByteToTransmit, uint8_t* pReceivedByte);
void L6474_Board_EnableIrq();
void L6474_Board_DisableIrq();
