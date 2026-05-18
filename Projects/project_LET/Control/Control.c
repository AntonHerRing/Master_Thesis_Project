#pragma GCC optimize ("O0") /* Incldue for dubuggning. Easier viewing of variables */
#include "Control.h"

//L6474_GetAcceleration(0) // get acceleration from stepper motor

//struct PID Pid1;
//struct PID *PID1 = &Pid1;

#define T_Enc   2
#define T_Motor 2
#define T_Contr 2   //2
#define T_Print 50  //50
#define T_Btns  2

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

float limit_value(float signal, float min, float max){
	if(signal < min)
		return min;
	else if(signal > max)
		return max;
	else
		return signal;
}

float max(float signal1, float signal2){
	if(signal1 > signal2){
		//printf("Min:: Sig1: %f > Sig2: %f\n", signal1, signal2);
		return signal1;
	}
	else{
		return signal2;
		//printf("Min:: Sig1: %f < Sig2: %f\n", signal1, signal2);
	}
}

float min(float signal1, float signal2){
	if(signal1 < signal2){
		//printf("Min:: Sig1: %f < Sig2: %f\n", signal1, signal2);
		return signal1;
	}
	else{
		return signal2;
		//printf("Min:: Sig1: %f > Sig2: %f\n", signal1, signal2);
	}
}

float sign(float signal){
	if(signal >= 0)
		return 1;
	else
		return -1;
}

void pid_filter_control_execute(arm_pid_instance_a_f32 *PID, float *current_error,
								float sample_period, float *Deriv_Filt) {

	float int_term, diff, diff_filt, contr_sig;
	static bool first_time = true;
	float error = *current_error;

	// Prevent derivative kick. Set curr and prev value as same.
	if (first_time && error != 0){
		PID->state_a[0] = error;
		first_time = false;
	}

	/* Compute time integral of error by trapezoidal rule */
	PID->int_term += (sample_period)*(error + PID->state_a[0])/2;

	/* Compute time derivative of error */
	diff = PID->Kd*(error - PID->state_a[0])/(sample_period);

	/*
	* Compute first order low pass filter of time derivative. IIR filter
	* Filter_out = feedforward_gain * (Deriv + past_Deriv) - feedback_term*Past_Filter_out
	* feedback_term is the pole location
	*/
	if (PID->Kd != 0)
		STM_Lowpass(diff, PID->state_a[2], Deriv_Filt[0], Deriv_Filt[1], PID->state_a[3], &diff_filt);
	else
		diff_filt = 0;

	/* Accumulate PID output with Integral, Derivative and Proportional contributions*/
	contr_sig = PID->Kp*error + PID->Ki*PID->int_term + diff_filt;
	PID->control_output = contr_sig;

	//printf("Error: %f\tdiff: %f\tdiff_filt: %f\toutput: %f\n", ((current_error) - PID->state_a[0]), diff, diff_filt, PID->control_output);

	/* Update state variables */
	PID->state_a[0] = error;			//e(t - 1)
	PID->state_a[1] = PID->state_a[0];	//e(t - 2)
	PID->state_a[2] = diff;
	PID->state_a[3] = diff_filt;
}

// RC term might falsly appear to make the filter work.
void pid_filter_control_executeV2(arm_pid_instance_a_f32 *PID, float *current_error,
									float sample_period, float cutoff_freq) {

	float int_term, diff, diff_filt, contr_sig;
	static bool first_time = true;
	float error = *current_error;

	// Prevent derivative kick. Set curr and prev value as same.
	if (first_time && error != 0){
		PID->state_a[0] = error;
		first_time = false;
	}

	/* Compute time integral of error by trapezoidal rule */
	PID->int_term += (sample_period)*(error + PID->state_a[0])/2;
	if(PID->Ki != 0){										//clamp the value
		PID->int_term = limit_value(PID->Ki*PID->int_term, -60, 60)/PID->Ki;
	}

	diff = (error - PID->state_a[0])/sample_period;

	/*
	* Compute first order low pass filter of time derivative. IIR filter
	* Filter_out = feedforward_gain * (Deriv + past_Deriv) - feedback_term*Past_Filter_out
	* feedback_term is the pole location
	*/

	diff_filt = diff;


	//contr_sig =  PID->Kd*diff_filt + PID->Ki*int_term + PID->Kp*error;
	contr_sig =  PID->Kd*diff_filt + PID->Ki*PID->int_term + PID->Kp*error;

	PID->control_output = contr_sig;

	//printf("Cutoff: %f\tError: %f\tdiff: %f\tdiff_filt: %f\toutput: %f\n", cutoff_freq, (error - PID->state_a[0]), diff, diff_filt, PID->control_output);

	/* Update state variables */
	PID->state_a[0] = error;			//e(t - 1)
	PID->state_a[1] = PID->state_a[0];	//e(t - 2)
	PID->state_a[2] = diff;
	PID->state_a[3] = diff_filt;
}

void pid_filter_control_execute_Incremental(inverted_pid_contr *PID, float *current_error, float sample_period) {

	float Delt_int, deriv_term, diff_filt, Delt_deriv, contr_sig;
	static bool first_time = true;
	float err = *current_error;

	/* Rate Limitors*/
	//float contr_min = max(-180, PID->prev_control_output_sat + sample_period*(-25.0));
	//float contr_max = min(180, 	PID->prev_control_output_sat + sample_period*(25.0));

	// Filter process variable
	float Delt_err = err - PID->prev_error_1; 	// Δe(t) = e(t) - Δe(t - Δt)

	float proportional = PID->Kp*Delt_err;

	/* Compute time integral of error by trapezoidal rule */
	Delt_int = PID->Ki*sample_period*(err + PID->prev_error_1)/2.0f;

	//deriv_term = PID->Kd*(err - PID->prev_error_1)/sample_period;
	deriv_term = PID->Kd*(PID->measurment - PID->prev_measurment)/sample_period;
	/*deriv_term = (2.0f * PID->Kd*(PID->measurment - PID->prev_measurment)
			+ (2.0f * PID->tau - sample_period) * PID->prev_diff)								
			/ (2.0f * PID->tau + sample_period);*/

	Delt_deriv = deriv_term - PID->prev_diff;
	if (PID->Kd != 0)
		STM_Lowpass_simp(Delt_deriv, PID, &diff_filt);
	else
		diff_filt = 0;

	//lowpass_V2(Delt_deriv, &Deriv_Filt[2], &Deriv_Filt[1], sample_period, Deriv_Filt[0]);

	contr_sig =  PID->prev_control_output + proportional + Delt_int + diff_filt;

	/* Integration anti-windup */
	/*float contr_sat = max(min(contr_sig, contr_max), contr_min);
	//contr_sig -= (sample_period/PID->tau)*(contr_sig - contr_sat);
	if(Delt_int*(contr_sig - contr_sat) > 0){
		contr_sig -= sign(contr_sig - contr_sat)*min(abs(Delt_int), abs(contr_sig - contr_sat));
	}
	contr_sig -= min((sample_period/PID->tau), 1)*(contr_sig - contr_sat);*/

	PID->prev_control_output = contr_sig; 		// u(t - 1), save past output // xu

	/* Saturate Output*/
	//contr_sig = max(min(contr_sig, contr_max), contr_min);

	PID->control_output = contr_sig;	// u(t)	Write output
	//PID->control_output = max(min(contr_sig, contr_max), contr_min);	// u(t)	Write output

	/* Update state variables */
	PID->prev_measurment = PID->measurment;
	PID->prev_error_2 = PID->prev_error_1;	//e(t - 2)
	PID->prev_error_1 = err;				//e(t - 1)
	PID->prev_diff = deriv_term;
	PID->prev_filt = diff_filt;
	PID->prev_control_output_sat = contr_sig; 	// xus

	printf("int_term: %f\tError: %f\tderiv_term: %f\tFilt_deriv: %f\toutput: %f\n", PID->Ki*Delt_int, Delt_err, PID->Kd*Delt_deriv, diff_filt ,PID->control_output/STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE);
}

/******************************************************//**
 * @brief  Low Pass Filter for Derivative Mode
 * @param[in] deriv current derivative value
 * @param[in] prev_deriv past derivative value
 * @param[in] dt sample time
 * @param[in] RC RC = Tau (time constant)
 * @retval Lowpassed time deriv of Input.
 **********************************************************/
float lowpass(float deriv, float prev_deriv, float dt, float RC){
	float alpha = dt / (RC + dt);
	//y[0] = alpha * x[0];
	//for (int i = 2; i < len; i++){
		//y[i] = alpha * x[i] + (1 - alpha) * y[i - 1];
		//y[1] = alpha * x[1] + (1 - alpha) * y[0];
		//y[1] = alpha * x[1] + (1 - alpha) * alpha * x[0];
		//y[1] = alpha * (x[1] + (1 - alpha)*x[0]);
	//}
	return alpha * (deriv + (1 - alpha)*prev_deriv);
}

float lowpass_alt(float deriv, float prev_out, float dt, float RC){
	float alpha = dt / (RC + dt);
	return alpha*deriv + (1 - alpha)*prev_out;
}

void lowpass_V2(float input, float *prev_out, float *out, float dt, float TC){
	float alpha = dt / (TC + 0.5*dt);

	*prev_out 	+= alpha*(input - *prev_out);
	*out 		+= alpha*(*prev_out - *out);
}

void STM_Lowpass(float input, float prev_in, float ff_gain, float fb_gain, float prev_out, float *out){
	*out = ff_gain*(input + prev_in) + fb_gain*prev_out;
}

void STM_Lowpass_simp(float diff, inverted_pid_contr *PID, float *out){
	*out = PID->ff_gain*(diff + PID->prev_diff) + PID->fb_gain*PID->prev_filt;
}
