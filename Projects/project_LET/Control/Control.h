#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "bsp.h"
#include "math.h"

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

#define SETPOINT_WEIGHT_PEND        0  
#define SETPOINT_WEIGHT_ROTOR       0.0432 //    0.0434 > x > 0.0433 <- close

#define PRIMARY_PROPORTIONAL_MODE_1 0.05    //0.01  //0.012  //0.247    //0.20   
#define PRIMARY_INTEGRAL_MODE_1     58      //70//145//145     //140   //90//95    //160      //40 is a start      
#define PRIMARY_DERIVATIVE_MODE_1   0.002       //0.01//0.05//0.01//0.01       //0.00001    

#define SECONDARY_PROPORTIONAL_MODE_1 	-0.08//-0.08  //0.02        //0.5//0.07//0.07//0.05  //0.44    //0.02        
#define SECONDARY_INTEGRAL_MODE_1     	0.03//0.03  //0.5//0.05         //0.01        //4.75//5 <- Is VERY close   
#define SECONDARY_DERIVATIVE_MODE_1   	-0.0165//-0.015     -0.017 <- keeps it from growing. 

#define scale 10

#define DERIVATIVE_LOW_PASS_CORNER_FREQUENCY        (1.0 * scale)  		//0.1 can work 10 - Corner frequency of low pass filter of Primary PID derivative
#define DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR  (5.0 * scale)	// 50 - Corner frequency of low pass filter of Secondary PID derivative

//#define DERIVATIVE_LOW_PASS_CORNER_FREQUENCY 10  		// 10 - Corner frequency of low pass filter of Primary PID derivative
//#define DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR 50 	// 50 - Corner frequency of low pass filter of Secondary PID derivative

#define LP_CORNER_FREQ_ROTOR 100 						// 100 - Corner frequency of low pass filter of Rotor Angle
#define LP_CORNER_FREQ_STEP 50	

#define ENCODER_ANGLE_POLARITY -1.0				// Note that physical system applies negative polarity to pendulum angle
												// by definition of coordinate system.

#define CONTROLLER_GAIN_SCALE 						1
#define STEPPER_READ_POSITION_STEPS_PER_DEGREE 		8.888889	//	Stepper position read value in steps per degree
#define Rotor_scale 1.0f/STEPPER_READ_POSITION_STEPS_PER_DEGREE
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
  /*Set Point and measurment*/
  float Set_point;
  float measurment;

  float tau;

  /* Control Variables*/
  float Kp;          /** The proportional gain. */
  float Ki;          /** The integral gain. */
  float Kd;          /** The derivative gain. */

  /* Set point handling*/
  float b;
  float b_1;  // set to one only when no integral.
  float c;

  /* Derivative Filter*/
  float ff_gain;
  float fb_gain;

  /* The integral collector*/
  float int_term;
  
  /* The controller output */
  float control_output; 
  float prev_control_output; 
  float prev_control_output_sat; 

  /* Previous I/Os*/
  float prev_measurment;
  float prev_set_point;
  float prev_error_1;
  float prev_error_2;
  float prev_diff;
  float prev_filt;

  /* Anti-Windup*/
  bool clamp_on;
  float low_clamp;
  float high_clamp;
  float pre_sat;

} inverted_pid_contr;

typedef struct
{
  float state_a[5];  /** The filter state array of length 5. */
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

void pid_filter_control_execute(arm_pid_instance_a_f32 *PID, float *current_error,
		                            float sample_period, float * Deriv_Filt);

void pid_filter_control_executeV2(arm_pid_instance_a_f32 *PID, float *current_error,
								float sample_period, float cutoff_freq);

float lowpass(float error, float prev_error, float dt, float RC);
float lowpass_alt(float deriv, float prev_out, float dt, float RC);
void lowpass_V2(float input, float *prev_out, float *out,float dt, float TC);
void pid_filter_control_executeV3(inverted_pid_contr *PID, float *current_error, float sample_period);
void STM_Lowpass(float input, float prev_in, float ff_gain, float fb_gain, float prev_out, float *out);
void STM_Lowpass_simp(float diff, inverted_pid_contr *PID, float *out);

void pid_filter_control_execute_Incremental(inverted_pid_contr *PID, float *current_error,
									float sample_period);

float max(float signal1, float signal2);
float min(float signal1, float signal2);
float limit_value(float signal, float min, float max);