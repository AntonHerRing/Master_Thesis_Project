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

#include "Drivers/L16474/motor_rpi3b_interface.h"
#include "Drivers/L16474/l6474.h"
#include "Drivers/L16474/steppermotor.h"

/******************* Defines *******************/

// static variables for the pendulum
#define g 9.81  //m/s^2
#define l 0.235 //m --> Pendulum Arm
#define r 0.14 //m  --> Rotor Arm

#define PI 3.141592654


// From STM example
/*#define PRIMARY_PROPORTIONAL_MODE_1 300 
#define PRIMARY_INTEGRAL_MODE_1     0.0
#define PRIMARY_DERIVATIVE_MODE_1   30

#define SECONDARY_PROPORTIONAL_MODE_1 	15.0
#define SECONDARY_INTEGRAL_MODE_1     	0.0
#define SECONDARY_DERIVATIVE_MODE_1   	7.5*/

// Scaled down to Degrees maybe?
/*#define PRIMARY_PROPORTIONAL_MODE_1 33.75 
#define PRIMARY_INTEGRAL_MODE_1     0.0
#define PRIMARY_DERIVATIVE_MODE_1   3.350

#define SECONDARY_PROPORTIONAL_MODE_1 	15.0
#define SECONDARY_INTEGRAL_MODE_1     	0.0
#define SECONDARY_DERIVATIVE_MODE_1   	7.5*/

#define PRIMARY_PROPORTIONAL_MODE_1 0.20   // 3 too much, 0.3  - 0.6   // 1.5    //2   //3.5
#define PRIMARY_INTEGRAL_MODE_1     160    //40 is a start      //10 works now       - 0     // 0    //1     //1
#define PRIMARY_DERIVATIVE_MODE_1   0 //0.1 too much       - 15    // 0.3    //1     //2

#define SECONDARY_PROPORTIONAL_MODE_1 	0.02    //0.44    //0.5         - 0.3   // 0.2 <- has to be at least one. Does not return other wise
#define SECONDARY_INTEGRAL_MODE_1     	10     //10          // 5   //0.75           - 0     // 0
#define SECONDARY_DERIVATIVE_MODE_1   	0   //              - 4     // 0.1

/**
 * Problem Encountered with Derivative values. 
 * When the difference between the current_error and current angle
 * becomes to large, the sample_time blows up the value in the 
 * order of thousands, or tens of thousands.
 * Problem occurs when stepper motor moves quickly from one position,
 * to the next position. EX ::
 * 
 * Curr_error = 0.1125, Current_angle = -46. Sample time 2ms
 * (-46-(0.1125))/0.002 = -23 056.25
 * Which overflows the output value, and Gives the stepper
 * motor a false movment
 * 
 * Dont use Derivative_Mode right now, and look for solution.
 **/

//Test Other group values
/*#define PRIMARY_PROPORTIONAL_MODE_1 0.3
#define PRIMARY_INTEGRAL_MODE_1     10
#define PRIMARY_DERIVATIVE_MODE_1   0

#define SECONDARY_PROPORTIONAL_MODE_1 	0.01
#define SECONDARY_INTEGRAL_MODE_1     	0.04
#define SECONDARY_DERIVATIVE_MODE_1   	0*/

//#define scale_factor 100
#define scale_factor 1

#define DERIVATIVE_LOW_PASS_CORNER_FREQUENCY        0.09  		//0.1 can work 10 - Corner frequency of low pass filter of Primary PID derivative
#define DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR  0.09 	// 50 - Corner frequency of low pass filter of Secondary PID derivative

//#define DERIVATIVE_LOW_PASS_CORNER_FREQUENCY 10  		// 10 - Corner frequency of low pass filter of Primary PID derivative
//#define DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR 50 	// 50 - Corner frequency of low pass filter of Secondary PID derivative

#define LP_CORNER_FREQ_ROTOR 100 						// 100 - Corner frequency of low pass filter of Rotor Angle
#define LP_CORNER_FREQ_STEP 50	

#define ENCODER_ANGLE_POLARITY -1.0				// Note that physical system applies negative polarity to pendulum angle
												// by definition of coordinate system.

#define CONTROLLER_GAIN_SCALE 						1
#define STEPPER_READ_POSITION_STEPS_PER_DEGREE 		8.888889	//	Stepper position read value in steps per degree
#define STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE 	STEPPER_READ_POSITION_STEPS_PER_DEGREE
#define ENCODER_READ_ANGLE_SCALE 					6.666667 // Angle Scale 6.66667 for 600 Pulse Per Rev Resolution Optical Encoder
#define FULL_STATE_FEEDBACK_SCALE 					1.00 // Scale factor for Full State Feedback Architecture

#define ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE 20		// Default 8. Amplitude of step cycle. Note: Peak-to-Peak amplitude is double this value
#define LP_CORNER_FREQ_LONG_TERM 				0.01	// Corner frequency of low pass filter - default to 0.001


/************ Structs and Variables ************/

struct PID {
    // PID Parameters
    float kp;
    float kd;
    float ki;

    // Control variables
    float integral;
    float error;
    float prev_error;
};

typedef struct
{
  float state_a[4];  /** The filter state array of length 4. */
  float Kp;          /** The proportional gain. */
  float Ki;          /** The integral gain. */
  float Kd;          /** The derivative gain. */
  float int_term;    /** The controller integral output */
  float control_output; /** The controller output */
} arm_pid_instance_a_f32;

/****************** Func Inits ******************/

bool oppositeSigns(int x, int y);
void init_pid(struct PID *PID1, struct PID *PID2);
void PID_controller(struct PID *Pid_in, float encoder_angle);

void pid_filter_control_execute(arm_pid_instance_a_f32 *PID, float current_error,
		                            float sample_period, float * Deriv_Filt);

void pid_filter_control_executeV2(arm_pid_instance_a_f32 *PID, float *current_error,
								float sample_period, float cutoff_freq);

float lowpass(float error, float prev_error, float dt, float RC);
float lowpass_alt(float deriv, float prev_out, float dt, float RC);