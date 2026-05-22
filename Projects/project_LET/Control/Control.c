//#pragma GCC optimize ("O0") /* Incldue for dubuggning. Easier viewing of variables */
#include "Control.h"


/*
 * Returns true if the two arguments have opposite sign, false if not
 * @retval bool
 */
bool oppositeSigns(int x, int y) {
    return ((x ^ y) < 0);
}

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
		return signal1;
	}
	else{
		return signal2;
	}
}

float min(float signal1, float signal2){
	if(signal1 < signal2){
		return signal1;
	}
	else{
		return signal2;
	}
}

float sign(float signal){
	if(signal >= 0)
		return 1;
	else
		return -1;
}


void pid_filter_control_execute(inverted_pid_contr *PID, float *current_error, float sample_period) {

	float proportional, diff, diff_filt, contr_sig;
	static bool first_time = true;
	float error = *current_error;

	/* Compute porportional part */								
	proportional = PID->Kp*error;	

	/* Compute time integral of error by trapezoidal rule */
	PID->int_term = PID->int_term + 0.5f * PID->Ki*(sample_period)*(error + PID->prev_error_1);

	/* Compute time derivative of measurment to avoid derivative kick*/
	diff = PID->Kd*(error - PID->prev_error_1)/(sample_period);
	
	/* Compute first order low pass filter of time derivative*/
	if (PID->Kd != 0) STM_Lowpass_simp(diff, PID, &diff_filt);
	else diff_filt = 0;
	
	/* Accumulate PID output with Integral, Derivative and Proportional contributions*/
	contr_sig = proportional + PID->int_term + diff_filt;
	PID->control_output = contr_sig;

	/* Save down past variables*/
    PID->prev_error_2    = PID->prev_error_1; 	//e(t - 2)
   	PID->prev_error_1    = error;				//e(t - 1)
    PID->prev_diff       = diff;
    PID->prev_filt       = diff_filt;
	//printf("Proport: %f\tint: %f\tdiff_filt: %f\toutput: %f\n", proportional*Rotor_scale, PID->int_term*Rotor_scale , diff_filt*Rotor_scale, PID->control_output*Rotor_scale);
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
