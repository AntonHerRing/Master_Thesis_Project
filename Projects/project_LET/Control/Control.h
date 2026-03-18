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


// From STM example
#define PRIMARY_PROPORTIONAL_MODE_1 300
#define PRIMARY_INTEGRAL_MODE_1     0.0
#define PRIMARY_DERIVATIVE_MODE_1   30

#define SECONDARY_PROPORTIONAL_MODE_1 	15.0
#define SECONDARY_INTEGRAL_MODE_1     	0.0
#define SECONDARY_DERIVATIVE_MODE_1   	7.5

#define DERIVATIVE_LOW_PASS_CORNER_FREQUENCY 10  		// 10 - Corner frequency of low pass filter of Primary PID derivative
#define LP_CORNER_FREQ_ROTOR 100 						// 100 - Corner frequency of low pass filter of Rotor Angle
#define DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR 50 	// 50 - Corner frequency of low pass filter of Secondary PID derivative
#define LP_CORNER_FREQ_STEP 50	

#define ENCODER_ANGLE_POLARITY -1.0				// Note that physical system applies negative polarity to pendulum angle
												// by definition of coordinate system.

#define CONTROLLER_GAIN_SCALE 						1
#define STEPPER_READ_POSITION_STEPS_PER_DEGREE 		8.888889	//	Stepper position read value in steps per degree
#define STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE 	STEPPER_READ_POSITION_STEPS_PER_DEGREE
#define ENCODER_READ_ANGLE_SCALE 					6.666667 // Angle Scale 6.66667 for 600 Pulse Per Rev Resolution Optical Encoder
#define FULL_STATE_FEEDBACK_SCALE 					1.00 // Scale factor for Full State Feedback Architecture

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
 void pid_filter_control_execute(arm_pid_instance_a_f32 *PID, float * current_error,
		float sample_period, float * Deriv_Filt);