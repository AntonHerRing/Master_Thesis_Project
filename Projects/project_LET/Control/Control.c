#include "Control.h"

//L6474_GetAcceleration(0) // get acceleration from stepper motor

//struct PID Pid1;
//struct PID *PID1 = &Pid1;

#define T_Enc   100
#define T_Motor 50
#define T_Contr 100
#define T_Print 100

/*float *current_error_steps, *current_error_rotor_steps;
float encoder_angle_slope_corr_steps;
float pendulum_position_command_steps;
float rotor_control_target_steps;
int rotor_position_steps;
float rotor_position_command_steps;
float feedforward_gain;
float encoder_position;*/


void init_pid(struct PID *PID1, struct PID *PID2){
    PID1->kp    = PRIMARY_PROPORTIONAL_MODE_1;
    PID1->ki    = PRIMARY_INTEGRAL_MODE_1;
    PID1->kd    = PRIMARY_DERIVATIVE_MODE_1;
    PID1->kd    = 0.0;
    PID1->error = 0.0;
    PID1->prev_error = 0.0;
    PID1->integral   = 0.0;

    PID2->kp    = SECONDARY_PROPORTIONAL_MODE_1;
    PID2->ki    = SECONDARY_INTEGRAL_MODE_1;
    PID2->kd    = SECONDARY_DERIVATIVE_MODE_1;
    PID2->kd    = 0.0;
    PID2->error = 0.0;
    PID2->prev_error = 0.0;
    PID2->integral   = 0.0;
}

/*
 * Returns true if the two arguments have opposite sign, false if not
 * @retval bool
 */
bool oppositeSigns(int x, int y) {
    return ((x ^ y) < 0);
}

void PID_controller(struct PID *Pid_in, float encoder_angle){}


void pid_filter_control_execute(arm_pid_instance_a_f32 *PID, float * current_error,
								float sample_period, float * Deriv_Filt) {

	float int_term, diff, diff_filt;

	/* Compute time integral of error by trapezoidal rule */
	int_term = PID->Ki*(sample_period)*((*current_error) + PID->state_a[0])/2;
	//printf("PID-Ki: %f\tsample_period: %f\tPID->state_a: %f\n ", PID->Ki, sample_period, PID->state_a[0]);
	printf("Current Error: %f\n ", (*current_error));

	/* Compute time derivative of error */
	diff = PID->Kd*((*current_error) - PID->state_a[0])/(sample_period);
	printf("PID-Kd: %f\tsample_period: %f\tPID->state_a[0]: %f\n ", PID->Kd, sample_period, PID->state_a[0]);

	/* Compute first order low pass filter of time derivative */
	diff_filt = Deriv_Filt[0] * diff
				+ Deriv_Filt[0] * PID->state_a[2]
				- Deriv_Filt[1] * PID->state_a[3];

	printf("Deriv[0]: %f\t[1]: %f\tPID->state_a[2]: %f\tPID->state_a[3]: %f\n ", Deriv_Filt[0], Deriv_Filt[1], PID->state_a[2], PID->state_a[3]);

	/* Accumulate PID output with Integral, Derivative and Proportional contributions*/

	printf("int_term: %f\tdiff: %f\tdiff_filt: %f\n ", int_term, diff, diff_filt);

	PID->control_output = diff_filt + int_term + PID->Kp*(*current_error);

	//printf("PID contr Output: %f\tCurr Err: %f\n ", PID->control_output, *current_error);

	/* Update state variables */
	PID->state_a[1] = PID->state_a[0];
	PID->state_a[0] = *current_error;
	PID->state_a[2] = diff;
	PID->state_a[3] = diff_filt;
	PID->int_term = int_term;
}

/*void test_task(int local_count){

    // CMSIS Variables 
    arm_pid_instance_a_f32 PID_Pend, PID_Rotor;
    float Deriv_Filt_Pend[2];
    float Deriv_Filt_Rotor[2];
    float Wo_t, fo_t, IWon_t;

    fo_t = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY;
	Wo_t = 2 * 3.141592654 * fo_t;
	IWon_t = 2 / (Wo_t * (T_Enc));
	Deriv_Filt_Pend[0] = 1 / (1 + IWon_t);
	Deriv_Filt_Pend[1] = Deriv_Filt_Pend[0] * (1 - IWon_t);

	fo_t = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR;
	Wo_t = 2 * 3.141592654 * fo_t;
	IWon_t = 2 / (Wo_t * (T_Motor));
	Deriv_Filt_Rotor[0] = 1 / (1 + IWon_t);
	Deriv_Filt_Rotor[1] = Deriv_Filt_Rotor[0] * (1 - IWon_t);

	*current_error_steps        = 0;
	*current_error_rotor_steps  = 0;
	PID_Pend.state_a[0] = 0;
	PID_Pend.state_a[1] = 0;
	PID_Pend.state_a[2] = 0;
	PID_Pend.state_a[3] = 0;
	PID_Pend.int_term   = 0;
	PID_Pend.control_output = 0;

	PID_Rotor.state_a[0]    = 0;
	PID_Rotor.state_a[1]    = 0;
	PID_Rotor.state_a[2]    = 0;
	PID_Rotor.state_a[3]    = 0;
	PID_Rotor.int_term      = 0;
	PID_Rotor.control_output = 0;

    encoder_angle_slope_corr_steps  = 0;
    pendulum_position_command_steps = 0;
    rotor_control_target_steps      = 0;
    rotor_position_steps            = 0;
    rotor_position_command_steps    = 0;
    feedforward_gain                = 1;
    encoder_position                = 0;

    while(1){
        // Initialize Pendulum PID control state 
        //ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
        encoder_position = local_count;

        pid_filter_control_execute(&PID_Pend, current_error_steps, T_Enc,
                Deriv_Filt_Pend);

		*current_error_rotor_steps = 0;
		pid_filter_control_execute(&PID_Rotor, current_error_rotor_steps,
				T_Motor, Deriv_Filt_Rotor);

        *current_error_steps = encoder_angle_slope_corr_steps
                + ENCODER_ANGLE_POLARITY * (encoder_position / ((float)(ENCODER_READ_ANGLE_SCALE/STEPPER_READ_POSITION_STEPS_PER_DEGREE)));

        //*current_error_steps = *current_error_steps + pendulum_position_command_steps;

        pid_filter_control_execute(&PID_Pend, current_error_steps, T_Enc, Deriv_Filt_Pend);

        //rotor_control_target_steps = PID_Pend.control_output;

    	pid_filter_control_execute(&PID_Rotor, current_error_rotor_steps, T_Motor,  Deriv_Filt_Rotor);

		rotor_control_target_steps = PID_Pend.control_output + PID_Rotor.control_output;

        /// Acquire rotor position and compute low pass filtered rotor position 

        //ret = rotor_position_read(&rotor_position_steps);

        //rotor_control_target_steps = rotor_control_target_steps - rotor_position_command_steps*feedforward_gain;

        BSP_MotorControl_GoTo(0, rotor_control_target_steps/2);

    }
}*/
