//#pragma GCC optimize ("O0") /* Incldue for dubuggning. Easier viewing of variables */
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
	//int_term = (sample_period)*(error + PID->state_a[0])/2;
	//PID->int_term += (sample_period)*(error + PID->state_a[0])/2;
	PID->int_term += sample_period*error;
	/*if(PID->Ki != 0){										//clamp the value
		PID->int_term = limit_value(PID->Ki*PID->int_term, -60, 60)/PID->Ki;
	}*/
	//int_term = PID->int_term + (sample_period)*((*current_error));

	/* Compute time derivative of error */
	//diff = ((*current_error) - PID->state_a[0])/(sample_period);
	diff = (error - PID->state_a[0])/(sample_period);
	//diff = error/sample_period;

	
	/* 
	* Compute first order low pass filter of time derivative. IIR filter 
	* Filter_out = feedforward_gain * (Deriv + past_Deriv) - feedback_term*Past_Filter_out
	* feedback_term is the pole location
	*/
	/*if (PID->Kd != 0)
		diff_filt = Deriv_Filt[0]*(diff + PID->state_a[2]) + Deriv_Filt[1]*PID->state_a[3];
	else 
		diff_filt = 0;*/
	diff_filt = diff;
	//printf("Deriv[0]: %f\t[1]: %f\tPID->state_a[2]: %f\tPID->state_a[3]: %f\n ", Deriv_Filt[0], Deriv_Filt[1], PID->state_a[2], PID->state_a[3]);

	/* Accumulate PID output with Integral, Derivative and Proportional contributions*/


	//PID->control_output = diff_filt + int_term + PID->Kp*(*current_error);
	//contr_sig =  PID->Kd*diff + PID->Ki*int_term + PID->Kp*(*current_error);
	contr_sig =  PID->Kd*diff_filt + PID->Ki*PID->int_term + PID->Kp*error;
	//PID->control_output = limit_value(contr_sig, -180, 180);
	PID->control_output = contr_sig;

	//printf("Rotor:: int_term: %f\tError: %f\tdiff: %f\tdiff_filt: %f\toutput: %f\n", PID->Ki*PID->int_term, (error - PID->state_a[0]), diff, PID->Kd*diff_filt, PID->control_output);
	//printf("Error: %f\tdiff: %f\tdiff_filt: %f\toutput: %f\n", ((current_error) - PID->state_a[0]), diff, diff_filt, PID->control_output);

	/* Update state variables */
	PID->state_a[0] = error;			//e(t - 1)
	PID->state_a[1] = PID->state_a[0];	//e(t - 2)
	PID->state_a[2] = diff;
	PID->state_a[3] = diff_filt;
	//PID->int_term = int_term;
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
	float RC = 1.0/(2.0*PI * cutoff_freq);

	/* Compute time integral of error by trapezoidal rule */
	//int_term = (sample_period)*((*current_error) + PID->state_a[0])/2;
	PID->int_term += (sample_period)*(error + PID->state_a[0])/2;
	if(PID->Ki != 0){										//clamp the value
		PID->int_term = limit_value(PID->Ki*PID->int_term, -60, 60)/PID->Ki;
	}

	diff = (error - PID->state_a[0])/sample_period;
	//diff = error/sample_period;
	if(PID->Kd != 0){											//clamp the value
		//diff = limit_value(PID->Kd*diff, -60, 60)/PID->Kd;
	}
	
	/* 
	* Compute first order low pass filter of time derivative. IIR filter 
	* Filter_out = feedforward_gain * (Deriv + past_Deriv) - feedback_term*Past_Filter_out
	* feedback_term is the pole location
	*/
	//diff_filt = lowpass(diff, PID->state_a[2], sample_period, RC);
	diff_filt = diff;
	//diff_filt = lowpass_alt(diff, PID->state_a[3], sample_period, RC);

	//contr_sig =  PID->Kd*diff_filt + PID->Ki*int_term + PID->Kp*error;
	contr_sig =  PID->Kd*diff_filt + PID->Ki*PID->int_term + PID->Kp*error;
	//PID->control_output = limit_value(contr_sig, -270, 270);
	PID->control_output = contr_sig;

	//printf("Pend:: int_term: %f\tError: %f\tdiff: %f\tdiff_filt: %f\toutput: %f\n", PID->Ki*PID->int_term, (error - PID->state_a[0]), diff, PID->Kd*diff_filt, PID->control_output);

	//printf("Cutoff: %f\tError: %f\tdiff: %f\tdiff_filt: %f\toutput: %f\n", cutoff_freq, (error - PID->state_a[0]), diff, diff_filt, PID->control_output);

	/* Update state variables */
	PID->state_a[0] = error;			//e(t - 1)
	PID->state_a[1] = PID->state_a[0];	//e(t - 2)
	PID->state_a[2] = diff;
	PID->state_a[3] = diff_filt;
	//PID->int_term = int_term;
}

void pid_filter_control_execute_Incremental(arm_pid_instance_a_f32 *PID, float *current_error,
									float sample_period, float *Deriv_Filt) {

	float Delt_int, deriv_term, Delt_deriv, contr_sig;
	static bool first_time = true;
	float err = *current_error;

	/* Rate Limitors*/
	float contr_min = max(-180, PID->state_a[3] + sample_period*(-25.0));
	float contr_max = min(180, 	PID->state_a[3] + sample_period*(25.0));

	// Prevent derivative kick. Set curr and prev value as same.
	if (first_time && err != 0){
		PID->state_a[0] = err;
		PID->state_a[1] = err;
		first_time = false;
	}
	// Filter process variable				
	float Delt_err = err - PID->state_a[0]; 	// Δe(t) = e(t) - Δe(t - Δt)

	/* Compute time integral of error by trapezoidal rule */
	Delt_int = sample_period*(err + PID->state_a[0])/2;

	deriv_term = (err - PID->state_a[0])/sample_period;
	Delt_deriv = deriv_term - PID->state_a[2];
	lowpass_V2(Delt_deriv, &Deriv_Filt[2], &Deriv_Filt[1], sample_period, Deriv_Filt[0]);	
	
	//contr_sig =  PID->state_a[4] + PID->Kp*Delt_err + PID->Ki*Delt_int + PID->Kd*Delt_deriv + bias;
	contr_sig =  PID->state_a[4] + PID->Kp*Delt_err + PID->Ki*Delt_int + PID->Kd*Deriv_Filt[1];

	/* Integration anti-windup */
	float contr_sat = max(min(contr_sig, contr_max), contr_min);
	//contr_sig -= (sample_period/Deriv_Filt[0])*(contr_sig - contr_sat);
	if(Delt_int*(contr_sig - contr_sat) > 0){
		contr_sig -= sign(contr_sig - contr_sat)*min(abs(Delt_int), abs(contr_sig - contr_sat));
	}
	contr_sig -= min((sample_period/Deriv_Filt[0]), 1)*(contr_sig - contr_sat);

	PID->state_a[4] = contr_sig; 		// u(t - 1), save past output // xu

	/* Saturate Output*/
	//contr_sig = max(min(contr_sig, contr_max), contr_min);

	PID->control_output = contr_sig;	// u(t)	Write output
	//PID->control_output = max(min(contr_sig, contr_max), contr_min);	// u(t)	Write output

	/* Update state variables */
	PID->state_a[0] = err;				//e(t - 1)
	PID->state_a[1] = PID->state_a[0];	//e(t - 2)
	PID->state_a[2] = deriv_term;			
	PID->state_a[3] = contr_sig; 	// xus

	printf("int_term: %f\tError: %f\tderiv_term: %f\tFilt_deriv: %f\tContr_sat: %f\toutput: %f\n", PID->Ki*Delt_int, Delt_err, PID->Kd*Delt_deriv, PID->Kd*Deriv_Filt[1], contr_sat ,PID->control_output);
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

void lowpass_V2(float input, float *prev_out, float *out,float dt, float TC){
	float alpha = dt / (TC + 0.5*dt);
	
	*prev_out 	+= alpha*(input - *prev_out);
	*out 		+= alpha*(*prev_out - *out);
}
