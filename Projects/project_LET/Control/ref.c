#include "ref.h"


/* Control system output signal */
float rotor_control_target_steps;
float rotor_control_target_steps_curr;
float rotor_control_target_steps_prev;

/* Control system variables */
int rotor_position_delta;
int initial_rotor_position;
int cycle_count;
int i, j, k, m;
int ret;

/* PID control system variables */
float windup, rotor_windup;
float *current_error_steps, *current_error_rotor_steps;
float *sample_period, *sample_period_rotor;

/* Loop timing measurement variables */
int cycle_period_start;
int cycle_period_sum;
int enable_cycle_delay_warning;

/* PID control variables */
float *deriv_lp_corner_f;
float *deriv_lp_corner_f_rotor;
float proportional, rotor_p_gain;
float integral, rotor_i_gain;
float derivative, rotor_d_gain;

/* State Feedback variables */
int enable_state_feedback;
float integral_compensator_gain;
float feedforward_gain;
float current_error_rotor_integral;

/* Reference tracking command */
float reference_tracking_command;

/* Pendulum position and tracking command */

/* Rotor position and tracking command */
int rotor_position_steps;
float rotor_position_command_steps;
float rotor_position_command_steps_pf, rotor_position_command_steps_pf_prev;
float rotor_position_command_deg;
float rotor_position_steps_prev, rotor_position_filter_steps, rotor_position_filter_steps_prev;
float rotor_position_diff, rotor_position_diff_prev;
float rotor_position_diff_filter, rotor_position_diff_filter_prev;
int rotor_target_in_steps;
int initial_rotor_position;

/* Rotor Plant Design variables */
int select_rotor_plant_design, enable_rotor_plant_design, enable_rotor_plant_gain_design;
int rotor_control_target_steps_int;
float rotor_damping_coefficient, rotor_natural_frequency;
float rotor_plant_gain;
float rotor_control_target_steps_gain;
float rotor_control_target_steps_filter_2, rotor_control_target_steps_filter_prev_2;
float rotor_control_target_steps_prev_prev, rotor_control_target_steps_filter_prev_prev_2;
float c0, c1, c2, c3, c4, ao, Wn2;
float fo_r, Wo_r, IWon_r, iir_0_r, iir_1_r, iir_2_r;

/* Encoder position variables */
uint32_t cnt3;
int range_error;
float encoder_position;
int encoder_position_steps;
int encoder_position_init;
int previous_encoder_position;
int max_encoder_position;
int global_max_encoder_position;
int prev_global_max_encoder_position;
int encoder_position_down;
int encoder_position_curr;
int encoder_position_prev;

/* Angle calibration variables */
float encoder_position_offset;
float encoder_position_offset_zero;
int enable_angle_cal;
int enable_angle_cal_resp;
int offset_end_state;
int offset_start_index;
int angle_index;
int angle_avg_index;
int angle_avg_span;
int offset_angle[ANGLE_CAL_OFFSET_STEP_COUNT + 2];
float encoder_position_offset_avg[ANGLE_CAL_OFFSET_STEP_COUNT + 2];
int angle_cal_end;
int angle_cal_complete;

/* Swing Up system variables */
int enable_swing_up;
int enable_swing_up_resp;
bool peaked;
bool handled_peak;
int zero_crossed;
motorDir_t swing_up_direction;
int swing_up_state, swing_up_state_prev;
int stage_count;
int stage_amp;

/* Initial control state parameter storage */
float init_r_p_gain, init_r_i_gain, init_r_d_gain;
float init_p_p_gain, init_p_i_gain, init_p_d_gain;
int init_enable_state_feedback;
float init_integral_compensator_gain;
float init_feedforward_gain;
int init_enable_state_feedback;
int init_enable_disturbance_rejection_step;
int init_enable_sensitivity_fnc_step;
int init_enable_noise_rejection_step;
int init_enable_rotor_plant_design;
int init_enable_rotor_plant_gain_design;

/* Low pass filter variables */
float fo, Wo, IWon, iir_0, iir_1, iir_2;
float fo_LT, Wo_LT, IWon_LT;
float iir_LT_0, iir_LT_1, iir_LT_2;
float fo_s, Wo_s, IWon_s, iir_0_s, iir_1_s, iir_2_s;

/* Slope correction system variables */
int slope;
int slope_prev;
float encoder_angle_slope_corr_steps;

/* Adaptive control variables */
float adaptive_error, adaptive_threshold_low, adaptive_threshold_high;
float error_sum_prev, error_sum, error_sum_filter_prev, error_sum_filter;
int adaptive_entry_tick, adaptive_dwell_period;
int enable_adaptive_mode, adaptive_state, adaptive_state_change;
float rotor_position_command_steps_prev;

/* Rotor impulse variables */
int rotor_position_step_polarity;
int impulse_start_index;

/* User configuration variables */
int clear_input;
uint32_t enable_control_action;
int max_speed_read, min_speed_read;
int select_suspended_mode;
int motor_response_model;
int enable_rotor_actuator_test, enable_rotor_actuator_control;
int enable_encoder_test;
int enable_rotor_actuator_high_speed_test;
int enable_motor_actuator_characterization_mode;
int motor_state;
float torq_current_val;


/* Rotor chirp system variables */
int enable_rotor_chirp;
int chirp_cycle;
int chirp_dwell_cycle;
float chirp_time;
float rotor_chirp_start_freq;
float rotor_chirp_end_freq;
float rotor_chirp_period ;
float rotor_chirp_frequency;
float rotor_chirp_amplitude;
int rotor_chirp_step_period;

float pendulum_position_command_steps;

/* Modulates sine tracking signal system variables */
int enable_mod_sin_rotor_tracking;
int enable_rotor_position_step_response_cycle;
int disable_mod_sin_rotor_tracking;
int sine_drive_transition;
float mod_sin_amplitude;
float rotor_control_sin_amplitude;
float rotor_sine_drive, rotor_sine_drive_mod;
float rotor_mod_control;
float mod_sin_carrier_frequency;

/* Pendulum impulse system variables */
int enable_pendulum_position_impulse_response_cycle;

/* Rotor high speed test system variables */
int swing_cycles, rotor_test_speed_min, rotor_test_speed_max;
int rotor_test_acceleration_max, swing_deceleration_max;
int start_angle_a[20], end_angle_a[20], motion_dwell_a[20];
int abs_encoder_position_prior, abs_encoder_position_after, abs_encoder_position_max;
uint16_t current_speed;

/*Pendulum system ID variable */
int enable_pendulum_sysid_test;

/* Full system identification variables */
int enable_full_sysid;
float full_sysid_max_vel_amplitude_deg_per_s;
float full_sysid_min_freq_hz;
float full_sysid_max_freq_hz;
int full_sysid_num_freqs;
float full_sysid_freq_log_step;
int full_sysid_start_index;

/* Rotor comb drive system variables */
int enable_rotor_tracking_comb_signal;
float rotor_track_comb_signal_frequency;
float rotor_track_comb_command;
float rotor_track_comb_amplitude;

/* Sensitivity function system variables */
int enable_disturbance_rejection_step;
int enable_noise_rejection_step;
int enable_plant_rejection_step;
int enable_sensitivity_fnc_step;
float load_disturbance_sensitivity_scale;

/* Noise rejection sensitivity function low pass filter */

float noise_rej_signal_filter, noise_rej_signal;
float noise_rej_signal_prev, noise_rej_signal_filter_prev;

/*
 * Real time user input system variables
 */

char config_message[16];
int config_command;
int display_parameter;
int step_size;
float adjust_increment;
int mode_index;

/* Real time data reporting index */
int report_mode;
int speed_scale;
int speed_governor;

/*
 * User selection mode values
 */

int mode_1;				// Enable LQR Motor Model M
int mode_2;				// Enable LRR Motor Model H
int mode_3;				// Enable LQR Motor Model L
int mode_4;				// Enable Suspended Mode Motor Model M
int mode_5;				// Enable sin drive track signal
int mode_adaptive_off;	// Disable adaptive control
int mode_adaptive;		// Enable adaptive control
int mode_8;				// Enable custom configuration entry
int mode_9;				// Disable sin drive track signal
int mode_10;			// Enable Single PID Mode with Motor Model M
int mode_11;			// Enable rotor actuator and encoder test mode
int mode_13;			// Enable rotor control system evaluation
int mode_15;			// Enable interactive control of rotor actuator
int mode_16;			// Enable load disturbance function step mode
int mode_17;			// Enable noise disturbance function step mode
int mode_18;			// Enable sensitivity function step mode
int mode_19;            // Enable full system identification mode
int mode_quit;			// Initiate exit from control loop
int mode_interactive;	// Enable continued terminal interactive user session
int mode_index_prev, mode_index_command;
int mode_transition_tick;
int mode_transition_state;
int transition_to_adaptive_mode;


/*
 * Real time user input characters
 */

char mode_string_stop[UART_RX_BUFFER_SIZE];
char mode_string_mode_1[UART_RX_BUFFER_SIZE];
char mode_string_mode_2[UART_RX_BUFFER_SIZE];
char mode_string_mode_3[UART_RX_BUFFER_SIZE];
char mode_string_mode_4[UART_RX_BUFFER_SIZE];
char mode_string_mode_8[UART_RX_BUFFER_SIZE];
char mode_string_mode_5[UART_RX_BUFFER_SIZE];
char mode_string_inc_accel[UART_RX_BUFFER_SIZE];
char mode_string_dec_accel[UART_RX_BUFFER_SIZE];
char mode_string_inc_amp[UART_RX_BUFFER_SIZE];
char mode_string_dec_amp[UART_RX_BUFFER_SIZE];
char mode_string_mode_single_pid[UART_RX_BUFFER_SIZE];
char mode_string_mode_test[UART_RX_BUFFER_SIZE];
char mode_string_mode_control[UART_RX_BUFFER_SIZE];
char mode_string_mode_motor_characterization_mode[UART_RX_BUFFER_SIZE];
char mode_string_mode_load_dist[UART_RX_BUFFER_SIZE];
char mode_string_mode_load_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_mode_noise_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_mode_plant_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_mode_full_sysid[UART_RX_BUFFER_SIZE];
char mode_string_dec_pend_p[UART_RX_BUFFER_SIZE];
char mode_string_inc_pend_p[UART_RX_BUFFER_SIZE];
char mode_string_dec_pend_i[UART_RX_BUFFER_SIZE];
char mode_string_inc_pend_i[UART_RX_BUFFER_SIZE];
char mode_string_dec_pend_d[UART_RX_BUFFER_SIZE];
char mode_string_inc_pend_d[UART_RX_BUFFER_SIZE];
char mode_string_dec_rotor_p[UART_RX_BUFFER_SIZE];
char mode_string_inc_rotor_p[UART_RX_BUFFER_SIZE];
char mode_string_dec_rotor_i[UART_RX_BUFFER_SIZE];
char mode_string_inc_rotor_i[UART_RX_BUFFER_SIZE];
char mode_string_dec_rotor_d[UART_RX_BUFFER_SIZE];
char mode_string_inc_rotor_d[UART_RX_BUFFER_SIZE];
char mode_string_dec_torq_c[UART_RX_BUFFER_SIZE];
char mode_string_inc_torq_c[UART_RX_BUFFER_SIZE];
char mode_string_dec_max_s[UART_RX_BUFFER_SIZE];
char mode_string_inc_max_s[UART_RX_BUFFER_SIZE];
char mode_string_dec_min_s[UART_RX_BUFFER_SIZE];
char mode_string_inc_min_s[UART_RX_BUFFER_SIZE];
char mode_string_dec_max_a[UART_RX_BUFFER_SIZE];
char mode_string_inc_max_a[UART_RX_BUFFER_SIZE];
char mode_string_dec_max_d[UART_RX_BUFFER_SIZE];
char mode_string_inc_max_d[UART_RX_BUFFER_SIZE];
char mode_string_enable_step[UART_RX_BUFFER_SIZE];
char mode_string_disable_step[UART_RX_BUFFER_SIZE];
char mode_string_enable_pendulum_impulse[UART_RX_BUFFER_SIZE];
char mode_string_disable_pendulum_impulse[UART_RX_BUFFER_SIZE];
char mode_string_enable_load_dist[UART_RX_BUFFER_SIZE];
char mode_string_disable_load_dist[UART_RX_BUFFER_SIZE];
char mode_string_enable_noise_rej_step[UART_RX_BUFFER_SIZE];
char mode_string_disable_noise_rej_step[UART_RX_BUFFER_SIZE];
char mode_string_disable_sensitivity_fnc_step[UART_RX_BUFFER_SIZE];
char mode_string_enable_sensitivity_fnc_step[UART_RX_BUFFER_SIZE];
char mode_string_inc_step_size[UART_RX_BUFFER_SIZE];
char mode_string_dec_step_size[UART_RX_BUFFER_SIZE];
char mode_string_select_mode_5[UART_RX_BUFFER_SIZE];
char mode_string_enable_high_speed_sampling[UART_RX_BUFFER_SIZE];
char mode_string_disable_high_speed_sampling[UART_RX_BUFFER_SIZE];
char mode_string_enable_speed_prescale[UART_RX_BUFFER_SIZE];
char mode_string_disable_speed_prescale[UART_RX_BUFFER_SIZE];
char mode_string_disable_speed_governor[UART_RX_BUFFER_SIZE];
char mode_string_enable_speed_governor[UART_RX_BUFFER_SIZE];
char mode_string_reset_system[UART_RX_BUFFER_SIZE];


int char_mode_select;	// Flag detecting whether character mode select entered


char message_received[UART_RX_BUFFER_SIZE];
char mode_string_mode_1[UART_RX_BUFFER_SIZE];
char mode_string_mode_2[UART_RX_BUFFER_SIZE];
char mode_string_mode_3[UART_RX_BUFFER_SIZE];
char mode_string_mode_4[UART_RX_BUFFER_SIZE];
char mode_string_mode_5[UART_RX_BUFFER_SIZE];
char mode_string_mode_8[UART_RX_BUFFER_SIZE];
char mode_string_mode_single_pid[UART_RX_BUFFER_SIZE];
char mode_string_mode_test[UART_RX_BUFFER_SIZE];
char mode_string_mode_control[UART_RX_BUFFER_SIZE];
char mode_string_mode_high_speed_test[UART_RX_BUFFER_SIZE];
char mode_string_mode_motor_characterization_mode[UART_RX_BUFFER_SIZE];
char mode_string_mode_pendulum_sysid_test[UART_RX_BUFFER_SIZE];
char mode_string_dec_accel[UART_RX_BUFFER_SIZE];
char mode_string_inc_accel[UART_RX_BUFFER_SIZE];
char mode_string_inc_amp[UART_RX_BUFFER_SIZE];
char mode_string_dec_amp[UART_RX_BUFFER_SIZE];
char mode_string_mode_load_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_mode_noise_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_mode_plant_dist_step[UART_RX_BUFFER_SIZE];
char mode_string_stop[UART_RX_BUFFER_SIZE];

/* CMSIS Variables */
arm_pid_instance_a_f32 PID_Pend, PID_Rotor;
float Deriv_Filt_Pend[2];
float Deriv_Filt_Rotor[2];
float Wo_t, fo_t, IWon_t;

/* System timing variables */

uint32_t tick, tick_cycle_current, tick_cycle_previous, tick_cycle_start,
tick_read_cycle, tick_read_cycle_start,tick_wait_start,tick_wait;

volatile uint32_t current_cpu_cycle, prev_cpu_cycle, last_cpu_cycle, target_cpu_cycle, prev_target_cpu_cycle;
volatile int current_cpu_cycle_delay_relative_report;

uint32_t t_sample_cpu_cycles;
float Tsample, Tsample_rotor, test_time;
float angle_scale;
int enable_high_speed_sampling;

/* Reset state tracking */
int reset_state;

/* Motor configuration */
uint16_t min_speed, max_speed, max_accel, max_decel;

/* Serial interface variables */
uint32_t RxBuffer_ReadIdx;
uint32_t RxBuffer_WriteIdx;
uint32_t readBytes;



int main(void) {

	/* Initialize reset state indicating that reset has occurred */

	reset_state = 1;

	/* initialize Integrator Mode time variables */
	apply_acc_start_time = 0;
	clock_int_time = 0;
	clock_int_tick = 0;

	/* Initialize PWM period variables used by step interrupt */
	desired_pwm_period = 0;
	current_pwm_period = 0;
	target_velocity_prescaled = 0;

	/* Initialize default start mode and reporting mode */
	mode_index = 1;
	report_mode = 1;

	/*Initialize serial read variables */
	RxBuffer_ReadIdx = 0;
	RxBuffer_WriteIdx = 0;
	readBytes = 0;

	/*Initialize encoder variables */
	encoder_position = 0;
	encoder_position_down = 0;
	encoder_position_curr = 0;
	encoder_position_prev = 0;
	angle_scale = ENCODER_READ_ANGLE_SCALE;

	/*Initialize rotor control variables */
	rotor_control_target_steps = 0;
	rotor_control_target_steps_curr = 0;
	rotor_control_target_steps_prev = 0;

	/*Initialize rotor plant design transfer function computation variables */
	rotor_control_target_steps_filter_prev_2 = 0.0;
	rotor_control_target_steps_filter_prev_prev_2 = 0.0;
	rotor_control_target_steps_prev_prev = 0.0;

	/* Initialize LQR integral control variables */
	current_error_rotor_integral = 0;

	/*Initialize rotor tracking signal variables */
	enable_rotor_chirp = 0;
	rotor_chirp_start_freq = ROTOR_CHIRP_START_FREQ;
	rotor_chirp_end_freq = ROTOR_CHIRP_END_FREQ;
	rotor_chirp_period = ROTOR_CHIRP_PERIOD;
	enable_mod_sin_rotor_tracking = ENABLE_MOD_SIN_ROTOR_TRACKING;
	enable_rotor_position_step_response_cycle = ENABLE_ROTOR_POSITION_STEP_RESPONSE_CYCLE;
	disable_mod_sin_rotor_tracking = 0;
	sine_drive_transition = 0;
	mod_sin_amplitude = MOD_SIN_AMPLITUDE;
	rotor_control_sin_amplitude = MOD_SIN_AMPLITUDE;

	/*Initialize sensitivity function selection variables */
	enable_disturbance_rejection_step = 0;
	enable_noise_rejection_step = 0;
	enable_sensitivity_fnc_step = 0;
	enable_pendulum_position_impulse_response_cycle = 0;

	/*Initialize user adjustment variables */
	step_size = 0;
	adjust_increment = 0.5;

	/*Initialize adaptive mode state variables */
	mode_transition_state = 0;
	transition_to_adaptive_mode = 0;

	/*Initialize user interactive mode */
	char_mode_select = 0;



	/* Default select_suspended_mode */
	select_suspended_mode = ENABLE_SUSPENDED_PENDULUM_CONTROL;


	/* Default controller gains */
	proportional = PRIMARY_PROPORTIONAL_MODE_1;
	integral = PRIMARY_INTEGRAL_MODE_1;
	derivative = PRIMARY_DERIVATIVE_MODE_1;
	rotor_p_gain = SECONDARY_PROPORTIONAL_MODE_1;
	rotor_i_gain = SECONDARY_INTEGRAL_MODE_1;
	rotor_d_gain = SECONDARY_DERIVATIVE_MODE_1;

	/* Enable State Feedback mode and Integral Action Compensator by default and set
	 * precompensation factor to unity
	 */
	enable_state_feedback = 1;
	integral_compensator_gain = 0;
	feedforward_gain = 1;

	/* Disable adaptive_mode by default */
	enable_adaptive_mode = 0;

	/* Controller structure and variable allocation */
	current_error_steps = malloc(sizeof(float));
	current_error_rotor_steps = malloc(sizeof(float));
	sample_period = malloc(sizeof(float));
	deriv_lp_corner_f = malloc(sizeof(float));
	deriv_lp_corner_f_rotor = malloc(sizeof(float));
	sample_period_rotor = malloc(sizeof(float));

	/* Configure controller filter and sample time parameters */
	*deriv_lp_corner_f = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY;
	*deriv_lp_corner_f_rotor = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR;
	t_sample_cpu_cycles = (uint32_t) round(T_SAMPLE_DEFAULT * RCC_HCLK_FREQ);
	Tsample = (float) t_sample_cpu_cycles / RCC_HCLK_FREQ;
	*sample_period = Tsample;
	Tsample_rotor = Tsample;
	*sample_period_rotor = Tsample_rotor;

	/* PID Derivative Low Pass Filter Coefficients */

	fo_t = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY;
	Wo_t = 2 * 3.141592654 * fo_t;
	IWon_t = 2 / (Wo_t * (*sample_period));
	Deriv_Filt_Pend[0] = 1 / (1 + IWon_t);
	Deriv_Filt_Pend[1] = Deriv_Filt_Pend[0] * (1 - IWon_t);

	fo_t = DERIVATIVE_LOW_PASS_CORNER_FREQUENCY_ROTOR;
	Wo_t = 2 * 3.141592654 * fo_t;
	IWon_t = 2 / (Wo_t * (*sample_period));
	Deriv_Filt_Rotor[0] = 1 / (1 + IWon_t);
	Deriv_Filt_Rotor[1] = Deriv_Filt_Rotor[0] * (1 - IWon_t);

	/* Initialize real time clock */
	assert(RCC_SYS_CLOCK_FREQ == HAL_RCC_GetSysClockFreq());
	assert(RCC_HCLK_FREQ == HAL_RCC_GetHCLKFreq());

	/* Configure primary controller parameters */
	windup = PRIMARY_WINDUP_LIMIT;

	/* Configure secondary Rotor controller parameters */
	rotor_windup = SECONDARY_WINDUP_LIMIT;

	/* Compute Low Pass Filter Coefficients for Rotor Position filter and Encoder Angle Slope Correction */
	fo = LP_CORNER_FREQ_ROTOR;
	Wo = 2 * 3.141592654 * fo;
	IWon = 2 / (Wo * Tsample);
	iir_0 = 1 / (1 + IWon);
	iir_1 = iir_0;
	iir_2 = iir_0 * (1 - IWon);
	fo_s = LP_CORNER_FREQ_STEP;
	Wo_s = 2 * 3.141592654 * fo_s;
	IWon_s = 2 / (Wo_s * Tsample);
	iir_0_s = 1 / (1 + IWon_s);
	iir_1_s = iir_0_s;
	iir_2_s = iir_0_s * (1 - IWon_s);
	fo_LT = LP_CORNER_FREQ_LONG_TERM;
	Wo_LT = 2 * 3.141592654 * fo_LT;
	IWon_LT = 2 / (Wo_LT * Tsample);
	iir_LT_0 = 1 / (1 + IWon_LT);
	iir_LT_1 = iir_LT_0;
	iir_LT_2 = iir_LT_0 * (1 - IWon_LT);

	/*
	 * Primary Controller Mode Configuration Loop
	 *
	 * Outer Loop acquires configuration command
	 * Inner Loop includes control system
	 * Inner Loop exits to Outer Loop upon command or exceedance
	 * of rotor or pendulum angles
	 *
	 * Outer Loop provided user-selected system reset option during
	 * data entry by Serial Interface
	 *
	 */


	tick_read_cycle_start = HAL_GetTick();
	/*
	 * Request user input for mode configuration
	 */

	enable_adaptive_mode = ENABLE_ADAPTIVE_MODE;
	adaptive_threshold_low = ADAPTIVE_THRESHOLD_LOW;
	adaptive_threshold_high = ADAPTIVE_THRESHOLD_HIGH;
	adaptive_state = ADAPTIVE_STATE;
	adaptive_state_change = 0;
	adaptive_dwell_period = ADAPTIVE_DWELL_PERIOD;

	while (1) {

		/* Set Motor Speed Profile and torque current */
		BSP_MotorControl_SoftStop(0);
		BSP_MotorControl_WaitWhileActive(0);
		L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);
		BSP_MotorControl_SetMaxSpeed(0, max_speed);
		BSP_MotorControl_SetMinSpeed(0, min_speed);
		BSP_MotorControl_SetAcceleration(0, MAX_ACCEL);
		BSP_MotorControl_SetDeceleration(0, MAX_DECEL);

		/*
		 * Configure Primary and Secondary PID controller data structures
		 * Scale by CONTROLLER_GAIN_SCALE set to default value of unity
		 */

		PID_Pend.state_a[0] = 0;
		PID_Pend.state_a[1] = 0;
		PID_Pend.state_a[2] = 0;
		PID_Pend.state_a[3] = 0;

		PID_Rotor.state_a[0] = 0;
		PID_Rotor.state_a[1] = 0;
		PID_Rotor.state_a[2] = 0;
		PID_Rotor.state_a[3] = 0;

		integral_compensator_gain = integral_compensator_gain * CONTROLLER_GAIN_SCALE;

		/* Assign Rotor Plant Design variable values */


		/* Transfer function model of form 1/(s^2 + 2*Damping_Coefficient*Wn*s + Wn^2) */
		if (rotor_damping_coefficient != 0 || rotor_natural_frequency != 0){
			Wn2 = rotor_natural_frequency * rotor_natural_frequency;
			rotor_plant_gain = rotor_plant_gain * Wn2;
			ao = ((2.0F/Tsample)*(2.0F/Tsample) + (2.0F/Tsample)*2.0F*rotor_damping_coefficient*rotor_natural_frequency
					+ rotor_natural_frequency*rotor_natural_frequency);
			c0 = ((2.0F/Tsample)*(2.0F/Tsample)/ao);
			c1 = -2.0F * c0;
			c2 = c0;
			c3 = -(2.0F*rotor_natural_frequency*rotor_natural_frequency - 2.0F*(2.0F/Tsample)*(2.0F/Tsample))/ao;
			c4 = -((2.0F/Tsample)*(2.0F/Tsample) - (2.0F/Tsample)*2.0F*rotor_damping_coefficient*rotor_natural_frequency
					+ rotor_natural_frequency*rotor_natural_frequency)/ao;
		}

		/* Transfer function model of form 1/(s^2 + Wn*s) */
		if (enable_rotor_plant_design == 2){
			IWon_r = 2 / (Wo_r * Tsample);
			iir_0_r = 1 - (1 / (1 + IWon_r));
			iir_1_r = -iir_0_r;
			iir_2_r = (1 / (1 + IWon_r)) * (1 - IWon_r);
		}


		/*
		 * *************************************************************************************************
		 *
		 * Control System Initialization Sequence
		 *
		 * *************************************************************************************************
		 */

		/* Setting enable_control_action enables control loop */
		enable_control_action = ENABLE_CONTROL_ACTION;

		/*
		 * Set Motor Position Zero occuring only once after reset and suppressed thereafter
		 * to maintain angle calibration
		 */

		if (reset_state == 1){
			rotor_position_set();
		}
		ret = rotor_position_read(&rotor_position_steps);
		sprintf(msg,
				"\r\nPrepare for Control Start - Initial Rotor Position: %i\r\n",
				rotor_position_steps);


		
		/*
		 * Initialize Pendulum Angle Read offset by setting encoder_position_init
		 */

		HAL_Delay(100);

		ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
		encoder_position_down = encoder_position_steps;
		sprintf(msg, "Pendulum Initial Angle %i\r\n", encoder_position_steps);


		/*
		 * Detect Start Condition for Pendulum Angle for Inverted Model
		 *
		 * Detect Pendulum Angle equal to vertical within tolerance of START_ANGLE
		 *
		 * Exit if no vertical orientation action detected and alert user to restart,
		 * then disable control and enable system restart.
		 *
		 * Permitted delay for user action is PENDULUM_ORIENTATION_START_DELAY.
		 *
		 */

		/*
		 * System start option with manual lifting of Pendulum to vertical by user
		 */


		/*
		 * Initialize Primary and Secondary PID controllers
		 */

		*current_error_steps = 0;
		*current_error_rotor_steps = 0;
		PID_Pend.state_a[0] = 0;
		PID_Pend.state_a[1] = 0;
		PID_Pend.state_a[2] = 0;
		PID_Pend.state_a[3] = 0;
		PID_Pend.int_term = 0;
		PID_Pend.control_output = 0;
		PID_Rotor.state_a[0] = 0;
		PID_Rotor.state_a[1] = 0;
		PID_Rotor.state_a[2] = 0;
		PID_Rotor.state_a[3] = 0;
		PID_Rotor.int_term = 0;
		PID_Rotor.control_output = 0;

		/* Initialize Pendulum PID control state */
		pid_filter_control_execute(&PID_Pend, current_error_steps, sample_period,
				 Deriv_Filt_Pend);

		/* Initialize Rotor PID control state */
		*current_error_rotor_steps = 0;
		pid_filter_control_execute(&PID_Rotor, current_error_rotor_steps,
				sample_period_rotor, Deriv_Filt_Rotor);

		/* Initialize control system variables */

		cycle_count = CYCLE_LIMIT;
		i = 0;
		rotor_position_steps = 0;
		rotor_position_steps_prev = 0;
		rotor_position_filter_steps = 0;
		rotor_position_filter_steps_prev = 0;
		rotor_position_command_steps = 0;
		rotor_position_diff = 0;
		rotor_position_diff_prev = 0;
		rotor_position_diff_filter = 0;
		rotor_position_diff_filter_prev = 0;
		rotor_position_step_polarity = 1;
		encoder_angle_slope_corr_steps = 0;
		rotor_sine_drive = 0;
		sine_drive_transition = 0;
		rotor_mod_control = 1.0;
		enable_adaptive_mode = 0;
		tick_cycle_start = HAL_GetTick();
		tick_cycle_previous = tick_cycle_start;
		tick_cycle_current =  tick_cycle_start;
		enable_cycle_delay_warning = ENABLE_CYCLE_DELAY_WARNING;
		chirp_cycle = 0;
		chirp_dwell_cycle = 0;
		pendulum_position_command_steps = 0;
		impulse_start_index = 0;
		mode_transition_state = 0;
		transition_to_adaptive_mode = 0;
		error_sum_prev = 0;
		error_sum_filter_prev = 0;
		adaptive_state = 4;
		rotor_control_target_steps_prev = 0;
		rotor_position_command_steps_prev = 0;
		rotor_position_command_steps_pf_prev = 0;
		enable_high_speed_sampling = ENABLE_HIGH_SPEED_SAMPLING_MODE;
		slope_prev = 0;
		rotor_track_comb_command = 0;
		noise_rej_signal_prev = 0;
		noise_rej_signal_filter_prev = 0;
		full_sysid_start_index = -1;
		current_cpu_cycle = 0;
		speed_scale = DATA_REPORT_SPEED_SCALE;
		speed_governor = 0;
		encoder_position_offset = 0;
		encoder_position_offset_zero = 0;

		for (m = 0; m < ANGLE_CAL_OFFSET_STEP_COUNT + 1; m++){
			offset_angle[m] = 0;
		}


		if(select_suspended_mode == 1){
			load_disturbance_sensitivity_scale = 1.0;
		}
/*
		 * *************************************************************************************************
		 *
		 * Control Loop Start
		 *
		 * *************************************************************************************************
		 */

		enable_control_action = 1;

		if (ACCEL_CONTROL == 1) {
			BSP_MotorControl_HardStop(0);
			L6474_CmdEnable(0);
			target_velocity_prescaled = 0;
			L6474_Board_SetDirectionGpio(0, BACKWARD);
		}

		/*
		 * Set Torque Current to value for normal operation
		 */
		torq_current_val = MAX_TORQUE_CONFIG;
		L6474_SetAnalogValue(0, L6474_TVAL, torq_current_val);

		ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
		if (select_suspended_mode == 0) {
			encoder_position = encoder_position_steps - encoder_position_down - (int)(180 * angle_scale);
			encoder_position = encoder_position - encoder_position_offset;
		}

		while (enable_control_action == 1) {

			/*
			 *  Real time user configuration and mode assignment
			 */



			/* Set mode 1 if user request detected */
			if (mode_index_command == 1 && mode_transition_state == 1) {
				mode_index = 1;
				mode_transition_state = 0;
				mode_index_command = 0;
				assign_mode_1(&PID_Pend, &PID_Rotor);
			}
			/* End of Real time user configuration and mode assignment read loop */


			/* Exit control if cycle count limit set */

			if (i > cycle_count && ENABLE_CYCLE_INFINITE == 0) {
				break;
			}

			/*
			 * *************************************************************************************************
			 *
			 * Initiate Measurement and Control
			 *
			 * *************************************************************************************************
			 */

			/*
			 * Acquire encoder position and correct for initial angle value of encoder measured at
			 * vertical down position at system start including 180 degree offset corresponding to
			 * vertical upwards orientation.
			 *
			 * For case of Suspended Mode Operation the 180 degree offset is not applied
			 *
			 * The encoder_position_offset variable value is determined by the Automatic Inclination
			 * Angle Calibration system
			 */

			ret = encoder_position_read(&encoder_position_steps, encoder_position_init, &htim3);
			if (select_suspended_mode == 1) {
				encoder_position = encoder_position_steps - encoder_position_down;
				encoder_position = encoder_position - encoder_position_offset;
			}

			/* Detect rotor position excursion exceeding limits and exit */

			if (rotor_position_steps
					> (ROTOR_POSITION_POSITIVE_LIMIT
							* STEPPER_READ_POSITION_STEPS_PER_DEGREE)) {
				sprintf(msg, "Error Exit Motor Position Exceeded: %i\r\n",
						rotor_position_steps);
				break;
			}

			/*
			 * Encoder Angle Error Compensation
			 *
			 * Compute Proportional control of pendulum angle compensating for error due to
			 * encoder offset at start time or system platform slope relative to horizontal.
			 *
			 * Apply optional time limit to correct for encoder error or slope.  For cycle count
			 * greater than ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT, encoder angle
			 * slope correction remains constant.
			 *
			 * If ENCODER_ANGLE_SLOPE_CORRECTION_CYCLE_LIMIT = 0, then encoder angle slope
			 * correction continues operation for all time
			 *
			 * Note: This system is *not* required in the event that Automatic Inclination
			 * Angle Calibration is selected.
			 *
			 */

			/* Compute Low Pass Filtered rotor position difference */

			rotor_position_diff_prev = rotor_position_diff;

			if (enable_disturbance_rejection_step == 0){
				rotor_position_diff = rotor_position_filter_steps
						- rotor_position_command_steps;
			}
			if (enable_disturbance_rejection_step == 1){
				rotor_position_diff = rotor_position_filter_steps;
			}

			/*
			 *  Compute current_error_steps input for Primary Controller
			 *
			 *  current_error_steps is sum of encoder angle error compensation and encoder position
			 *  in terms of stepper motor steps
			 *
			 *  An Encoder offset may be introduced.  The Encoder offset may remain at all times if
			 *  ENCODER_OFFSET_DELAY == 0 or terminate at a time (in ticks) of ENCODER_OFFSET_DELAY
			 */

			if ((i > ENCODER_START_OFFSET_DELAY) || (ENCODER_START_OFFSET_DELAY == 0)){
				encoder_position = encoder_position - ENCODER_START_OFFSET;
			}

			/*
			 * Compute error between Pendulum Angle and Pendulum Tracking Angle in units of steps
			 * Apply scale factor to match angle to step gain of rotor actuator
			 *
			 */

			*current_error_steps = encoder_angle_slope_corr_steps
			+ ENCODER_ANGLE_POLARITY * (encoder_position / ((float)(ENCODER_READ_ANGLE_SCALE/STEPPER_READ_POSITION_STEPS_PER_DEGREE)));

			/*
			 *
			 * Pendulum Controller execution
			 *
			 * Include addition of Pendulum Angle track signal impulse signal
			 * Compute control signal, rotor position target in step units
			 *
			 * Pendulum tracking command, pendulum_position_command, also supplied in step units
			 *
			 */

			*current_error_steps = *current_error_steps + pendulum_position_command_steps;

			pid_filter_control_execute(&PID_Pend,current_error_steps, sample_period, Deriv_Filt_Pend);

			rotor_control_target_steps = PID_Pend.control_output;

			/* Acquire rotor position and compute low pass filtered rotor position */


			ret = rotor_position_read(&rotor_position_steps);

			/*  Create rotor angle reference tracking modulated sine signal */

			rotor_sine_drive = 0;
			if (enable_mod_sin_rotor_tracking == 1 && ENABLE_ROTOR_CHIRP == 0 && i > angle_cal_complete) {

				if (ENABLE_ROTOR_CHIRP == 0){
					mod_sin_carrier_frequency = MOD_SIN_CARRIER_FREQ;
				}

				if (i > MOD_SIN_START_CYCLES && enable_mod_sin_rotor_tracking == 1) {
					rotor_sine_drive =
							(float) (mod_sin_amplitude
									* (1 + sin(-1.5707 + ((i - MOD_SIN_START_CYCLES)/MOD_SIN_SAMPLE_RATE) * (MOD_SIN_MODULATION_FREQ * 6.2832))));
					rotor_sine_drive_mod = sin(0 + ((i - MOD_SIN_START_CYCLES) /MOD_SIN_SAMPLE_RATE) * (mod_sin_carrier_frequency * 6.2832));
					rotor_sine_drive = rotor_sine_drive + MOD_SIN_MODULATION_MIN;
					rotor_sine_drive = rotor_sine_drive * rotor_sine_drive_mod * rotor_mod_control;
				}

				if ( fabs(rotor_sine_drive_mod*MOD_SIN_AMPLITUDE) < 2 && disable_mod_sin_rotor_tracking == 1 && sine_drive_transition == 1){
					rotor_mod_control = 0.0;
					sine_drive_transition = 0;
				}
				if ( fabs(rotor_sine_drive_mod*MOD_SIN_AMPLITUDE) < 2 && disable_mod_sin_rotor_tracking == 0 && sine_drive_transition == 1){
					rotor_mod_control = 1.0;
					sine_drive_transition = 0;
				}

				if (enable_rotor_position_step_response_cycle == 0){
					rotor_position_command_steps = rotor_sine_drive;
				}

			}

			/*
			 * Create pendulum angle reference tracking impulse signal.  Polarity of impulse alternates
			 */

			if (enable_pendulum_position_impulse_response_cycle == 1 && i != 0 && i > angle_cal_complete) {

				if ((i % PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_INTERVAL) == 0) {
					if (select_suspended_mode == 1) {
						pendulum_position_command_steps =
								(float) PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE;
					}
					if (select_suspended_mode == 0) {
						pendulum_position_command_steps =
								(float) (PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_AMPLITUDE
										/PENDULUM_POSITION_IMPULSE_AMPLITUDE_SCALE);
					}
					chirp_cycle = 0;
					impulse_start_index = 0;
				}
				if (impulse_start_index
						> PENDULUM_POSITION_IMPULSE_RESPONSE_CYCLE_PERIOD) {
					pendulum_position_command_steps = 0;
				}
				impulse_start_index++;
				chirp_cycle++;
			}

			/*  Create rotor angle reference tracking  step signal */

			if ((i % ROTOR_POSITION_STEP_RESPONSE_CYCLE_INTERVAL) == 0 && enable_rotor_position_step_response_cycle == 1 && i > angle_cal_complete) {
				rotor_position_step_polarity = -rotor_position_step_polarity;
				if (rotor_position_step_polarity == 1){
					chirp_cycle = 0;
				}
			}

			if (enable_rotor_position_step_response_cycle == 1 && enable_rotor_tracking_comb_signal == 0 && i > angle_cal_complete) {
				if (STEP_RESPONSE_AMP_LIMIT_ENABLE == 1 && abs(rotor_sine_drive) > STEP_RESPONSE_AMP_LIMIT){
					chirp_cycle = chirp_cycle + 1;
				} else {
					if (enable_mod_sin_rotor_tracking == 1){
						rotor_position_command_steps = rotor_sine_drive + (float) ((rotor_position_step_polarity)
								* ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE
								* STEPPER_READ_POSITION_STEPS_PER_DEGREE);
					}
					if (enable_mod_sin_rotor_tracking == 0){
						rotor_position_command_steps_pf = (float) ((rotor_position_step_polarity)
								* ROTOR_POSITION_STEP_RESPONSE_CYCLE_AMPLITUDE
								* STEPPER_READ_POSITION_STEPS_PER_DEGREE);
					}
					chirp_cycle = chirp_cycle + 1;
				}
			}

			/*
			 * Rotor tracking reference, rotor_position_command_steps, is low pass filtered to prevent
			 * aliasing of measurement during operation of Real Time Workbench sampling that occurs at
			 * 50 Hz (in support of connected computing platform bandwidth limitations).  This filter
			 * application is not applied during selection of high speed sampling at 500 Hz.
			 */

			if (enable_rotor_position_step_response_cycle == 1 && enable_mod_sin_rotor_tracking == 0 && enable_rotor_tracking_comb_signal == 0 && i > angle_cal_complete){
				rotor_position_command_steps = rotor_position_command_steps_pf * iir_0_s
						+ rotor_position_command_steps_pf_prev * iir_1_s
						- rotor_position_command_steps_prev * iir_2_s;
				rotor_position_command_steps_pf_prev = rotor_position_command_steps_pf;
			}

			/*
			 *  Automatic Inclination Angle Calibration System
			 *
			 *  The Edukit system may be resting on a surface with a slight incline. This then
			 *  produces a Rotor Angle dependent error between the measurement of Pendulum Angle
			 *  and the angle corresponding to true vertical of the gravitational vector.
			 *
			 *  This system computed true vertical angle relative to the gravity vector for each
			 *  Rotor Step.  This provides an encoder_offset_angle calibration value for all
			 *  orientations of the Rotor.
			 */

			if (enable_angle_cal == 1){

				/*
				 * Angle Calibration system state values applied during Angle Calibration
				 * Period.  User selected system state values restored after Angle Calibration
				 */

				if (i == 1 && select_suspended_mode == 1){
					PID_Rotor.Kp = -23.86;
					PID_Rotor.Ki = 0;
					PID_Rotor.Kd = -19.2;
					PID_Pend.Kp = -293.2;
					PID_Pend.Ki = 0.0;
					PID_Pend.Kd = -41.4;
					enable_state_feedback = 1;
					integral_compensator_gain = -11.45;
					feedforward_gain = 1;
					rotor_position_command_steps = 0;
					current_error_rotor_integral = 0;
				}


				/* Initialize angle calibration variables */

				if (i == 1){
					offset_end_state = 0;
					offset_start_index = 4000;					// initial start index for sweep
					angle_index = ANGLE_CAL_OFFSET_STEP_COUNT;	// Number of angle steps
					angle_cal_end = INT32_MAX;
					angle_cal_complete = INT32_MAX;				// Allowed start time for stimulus signals
					encoder_position_offset_zero = 0;
				}

				if (offset_end_state == 0){
					/* Suspend loop delay warning since computation may lead to control loop cycle delay during
					 * period after measurement and during computation of smoothed offset data array
					 */
					enable_cycle_delay_warning = 0;
					/* Advance to upper angle of 90 degrees*/
					if (i > 1 && i < 4000){
						rotor_position_command_steps = (i/4000.0) * ANGLE_CAL_OFFSET_STEP_COUNT/2;
						offset_start_index = i + 4000;
					}
					/* Delay for time increment to avoid transient response in measurement.
					 * Acquire samples for time-average of offset
					 */
					if (i >= offset_start_index && i < (offset_start_index + 10)){
						//offset_angle[angle_index] = offset_angle[angle_index] + encoder_position;
						//offset_angle[angle_index] = encoder_position;
					}
					/* Compute time-averages offset and advance to next lower angle increment */
					if (i == offset_start_index + 10 && angle_index > 0){
						offset_angle[angle_index] = encoder_position;
						//offset_angle[angle_index] = offset_angle[angle_index]/10;
						angle_index = angle_index - 1;
						offset_start_index = offset_start_index + 10;
						rotor_position_command_steps = rotor_position_command_steps - 1;
					}

					/* Compute average encoder position offset over angle index range from ANGLE_AVG_SPAN to ANGLE_CAL_OFFSET_STEP_COUNT - ANGLE_AVG_SPAN */
					/* Suspend delay warning */

					if (angle_index >= 2*ANGLE_AVG_SPAN && angle_index < ANGLE_CAL_OFFSET_STEP_COUNT + 1){
						for (angle_avg_index = angle_index - 2*ANGLE_AVG_SPAN; angle_avg_index < (angle_index + 1); angle_avg_index++){
							encoder_position_offset_avg[angle_index] = 0;
									for (angle_avg_index = angle_index - ANGLE_AVG_SPAN; angle_avg_index < (1 + angle_index + ANGLE_AVG_SPAN); angle_avg_index++){
											encoder_position_offset_avg[angle_index] = encoder_position_offset_avg[angle_index] + offset_angle[angle_avg_index];
									}
									encoder_position_offset_avg[angle_index] = encoder_position_offset_avg[angle_index]/(float)(2*ANGLE_AVG_SPAN + 1);
						}
					}

					/* Restore rotor angle to zero degrees */
					if (angle_index == 0){
						rotor_position_command_steps = rotor_position_command_steps + 0.02*STEPPER_READ_POSITION_STEPS_PER_DEGREE;
					}
					/* Terminate offset measurement and initialize angle_cal_end at time of termination */
					if (rotor_position_command_steps >= 0 && angle_index == 0){
						offset_end_state = 1;
						angle_cal_end = i;
						/* Restore loop delay warning */
						enable_cycle_delay_warning = 1;
					}
				}
			}

			/* Apply offset angle for correction of pendulum angle according to rotor position */

			if (offset_end_state == 1 && i > angle_cal_end){
				/* Compute angle index corresponding to rotor position */
				angle_index = (int)((ANGLE_CAL_OFFSET_STEP_COUNT - 1)/2) + rotor_position_filter_steps;
				if (angle_index < ANGLE_AVG_SPAN ){
					angle_index = ANGLE_AVG_SPAN;
				}
				if (angle_index >  ANGLE_CAL_OFFSET_STEP_COUNT - ANGLE_AVG_SPAN ){
					angle_index =  ANGLE_CAL_OFFSET_STEP_COUNT - ANGLE_AVG_SPAN;
				}
				encoder_position_offset = 2.0 * encoder_position_offset_avg[angle_index];
			}

			/* Measure residual offset at zero rotor position */
			if (offset_end_state == 1 && i > angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING && i < angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING + ANGLE_CAL_ZERO_OFFSET_DWELL){
				encoder_position_offset_zero = encoder_position_offset_zero + encoder_position;
			}

			/* Correct offset angle array values for any residual offset */
			if (i == (angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING + ANGLE_CAL_ZERO_OFFSET_DWELL + 1)){
				encoder_position_offset_zero = encoder_position_offset_zero/ANGLE_CAL_ZERO_OFFSET_DWELL;
				for (angle_index = 0; angle_index < ANGLE_CAL_OFFSET_STEP_COUNT + 1; angle_index++){
					encoder_position_offset_avg[angle_index] = encoder_position_offset_avg[angle_index] + encoder_position_offset_zero;
				}
				angle_cal_complete = angle_cal_end + ANGLE_CAL_ZERO_OFFSET_SETTLING + ANGLE_CAL_ZERO_OFFSET_DWELL + 1 + ANGLE_CAL_COMPLETION;
			}

			/* Restore user selected system state configuration */
			if (offset_end_state == 1 && (enable_angle_cal == 1) && i == angle_cal_complete + 1){
				PID_Rotor.Kp = init_r_p_gain;
				PID_Rotor.Ki = init_r_i_gain;
				PID_Rotor.Kd = init_r_d_gain;
				PID_Pend.Kp = init_p_p_gain;
				PID_Pend.Ki = init_p_i_gain;
				PID_Pend.Kd = init_p_d_gain;
				current_error_rotor_integral = 0;
				enable_state_feedback = init_enable_state_feedback;
				integral_compensator_gain = init_integral_compensator_gain;
				feedforward_gain = init_feedforward_gain;
				enable_state_feedback = init_enable_state_feedback;
				enable_disturbance_rejection_step = init_enable_disturbance_rejection_step;
				enable_sensitivity_fnc_step = init_enable_sensitivity_fnc_step;
				enable_noise_rejection_step = init_enable_noise_rejection_step;
				enable_rotor_plant_design = init_enable_rotor_plant_design;
			}
			if (ENABLE_DUAL_PID == 1) {

				/*
				 * Secondary Controller execution including Sensitivity Function computation
				 */

				if (enable_state_feedback == 0 && enable_disturbance_rejection_step == 0 && enable_sensitivity_fnc_step == 0 && enable_noise_rejection_step == 0){
					*current_error_rotor_steps = rotor_position_filter_steps - rotor_position_command_steps;
				}
				if (enable_state_feedback == 0 && enable_disturbance_rejection_step == 1 && enable_sensitivity_fnc_step == 0 && enable_noise_rejection_step == 0){
					*current_error_rotor_steps = rotor_position_filter_steps;
				}
				if (enable_state_feedback == 0 && enable_disturbance_rejection_step == 0 && enable_sensitivity_fnc_step == 0 && enable_noise_rejection_step == 1){
					*current_error_rotor_steps = rotor_position_filter_steps + rotor_position_command_steps;
				}

				if (enable_state_feedback == 0 && enable_disturbance_rejection_step == 0 && enable_sensitivity_fnc_step == 1 && enable_noise_rejection_step == 0){
					*current_error_rotor_steps = rotor_position_filter_steps - rotor_position_command_steps;
				}

				/*
				 * Select Reference signal input location at input of controller for Dual PID architecture
				 * for Output Feedback Architecture or at output of controller and plant input for Full State
				 * Feedback Architecture
				 */

				if (enable_state_feedback == 1){
					*current_error_rotor_steps = rotor_position_filter_steps;
				}

				/*
				 * PID input supplied in units of stepper motor steps with tracking error determined
				 * as difference between rotor position tracking command and rotor position in units
				 * of stepper motor steps.
				 */

				pid_filter_control_execute(&PID_Rotor, current_error_rotor_steps,
						sample_period_rotor,  Deriv_Filt_Rotor);

				rotor_control_target_steps = PID_Pend.control_output + PID_Rotor.control_output;


				if (enable_state_feedback == 1 && integral_compensator_gain != 0){
					/*
					 * If integral action is included, state feedback plant input equals
					 * the time integral of difference between reference tracking signal and rotor angle,
					 * current_error_rotor_integral.  This is summed with the controller output, rotor_control_target_steps.
					 * The integral_compensator_gain as input by user includes multiplicative scale factor matching
					 * scaling of controller gain values.
					 */
					current_error_rotor_integral = current_error_rotor_integral + (rotor_position_command_steps*feedforward_gain - rotor_position_filter_steps)*(*sample_period_rotor);
					rotor_control_target_steps = rotor_control_target_steps - integral_compensator_gain*current_error_rotor_integral;
				}

				if (enable_state_feedback == 1 && integral_compensator_gain == 0){
					/*
					 * If integral compensator is not included, full state feedback plant input equals difference
					 * between control output and reference tracking signal with scale factor matching
					 * scaling of controller gain values.
					 *
					 * If Plant Design system is applied, the effects of small numerical error in transfer function
					 * computation is compensated for by removal of average offset error.
					 *
					 */

					rotor_control_target_steps = rotor_control_target_steps - rotor_position_command_steps*feedforward_gain;

				}


				/*
				 * Load Disturbance Sensitivity Function signal introduction with scale factor applied to increase
				 * amplitude of Load Disturbance signal to enhance signal to noise in measurement.  This scale factor
				 * then must be applied after data acquisition to compute proper Load Disturbance Sensitivity Function.
				 * Note that Load Disturbance Sensitivity Function value is typically less than -20 dB
				 *
				 */
				if (enable_disturbance_rejection_step == 1){
					rotor_control_target_steps = rotor_control_target_steps + rotor_position_command_steps * load_disturbance_sensitivity_scale;
				}
			}


			if (full_sysid_start_index != -1 && i >= full_sysid_start_index && i > angle_cal_complete) {
				float total_acc = 0;
				float t = (i - full_sysid_start_index) * Tsample;
				float w = full_sysid_min_freq_hz * M_TWOPI;
				for (int k_step = 0; k_step < full_sysid_num_freqs; k_step++) {
					float wave_value = w * cosf(w * t); // multiply acceleration wave by omega to keep consistent velocity amplitude
					total_acc += wave_value;
					w *= full_sysid_freq_log_step;
				}
				rotor_control_target_steps = ((full_sysid_max_vel_amplitude_deg_per_s/full_sysid_num_freqs) * total_acc * STEPPER_CONTROL_POSITION_STEPS_PER_DEGREE);
			}


			/*
			 *
			 * Plant transfer function design based on two stage first order high pass IIR
			 * filter structures applied to rotor_control_target_steps.
			 *
			 * Second order system computed at all cycle times to avoid transient upon switching between
			 * operating modes with and without Rotor Plant Design enabled
			 *
			 */


			if (rotor_damping_coefficient != 0 || rotor_natural_frequency != 0){

					rotor_control_target_steps_filter_2 = c0*rotor_control_target_steps + c1*rotor_control_target_steps_prev
							+ c2*rotor_control_target_steps_prev_prev + c3*rotor_control_target_steps_filter_prev_2
							+ c4*rotor_control_target_steps_filter_prev_prev_2;

					rotor_control_target_steps_prev_prev = rotor_control_target_steps_prev;
					rotor_control_target_steps_filter_prev_prev_2 = rotor_control_target_steps_filter_prev_2;
					rotor_control_target_steps_filter_prev_2 = rotor_control_target_steps_filter_2;
			}

			if ((enable_rotor_plant_design == 2 )){
				rotor_control_target_steps_filter_2 = iir_0_r*rotor_control_target_steps + iir_1_r*rotor_control_target_steps_prev
						- iir_2_r*rotor_control_target_steps_filter_prev_2;
				rotor_control_target_steps_filter_prev_2 = rotor_control_target_steps_filter_2;
			}


			/*
			 * Record current value of rotor_position_command tracking signal
			 * and control signal, rotor_control_target_steps for rotor position
			 * rotor position filters, rotor plant design, performance monitoring and adaptive control
			 */

			rotor_control_target_steps_prev = rotor_control_target_steps;
			rotor_position_command_steps_prev = rotor_position_command_steps;


			if (ACCEL_CONTROL == 1) {
				if (enable_rotor_plant_design != 0){
					rotor_control_target_steps_filter_2 = rotor_plant_gain*rotor_control_target_steps_filter_2;
					apply_acceleration(&rotor_control_target_steps_filter_2, &target_velocity_prescaled, Tsample);
				/* Applies if Rotor Gain defined */
				} else if (enable_rotor_plant_gain_design == 1){
					rotor_control_target_steps_gain = rotor_plant_gain * rotor_control_target_steps;
					apply_acceleration(&rotor_control_target_steps_gain, &target_velocity_prescaled, Tsample);
				/* Applies if no Rotor Design is selected */
				} else {
					apply_acceleration(&rotor_control_target_steps, &target_velocity_prescaled, Tsample);
				}
			} else {
				BSP_MotorControl_GoTo(0, rotor_control_target_steps/2);
			}
		}

		/*
		 * *************************************************************************************************
		 *
		 * Control Loop Exit
		 *
		 * *************************************************************************************************
		 */
	}
}

 void pid_filter_control_execute(arm_pid_instance_a_f32 *PID, float * current_error,
		float * sample_period, float * Deriv_Filt) {

		float int_term, diff, diff_filt;

	  /* Compute time integral of error by trapezoidal rule */
	  int_term = PID->Ki*(*sample_period)*((*current_error) + PID->state_a[0])/2;

	  /* Compute time derivative of error */
	  diff = PID->Kd*((*current_error) - PID->state_a[0])/(*sample_period);

	  /* Compute first order low pass filter of time derivative */
	  diff_filt = Deriv_Filt[0] * diff
				+ Deriv_Filt[0] * PID->state_a[2]
				- Deriv_Filt[1] * PID->state_a[3];

	  /* Accumulate PID output with Integral, Derivative and Proportional contributions*/

	  PID->control_output = diff_filt + int_term + PID->Kp*(*current_error);

	  /* Update state variables */
	  PID->state_a[1] = PID->state_a[0];
	  PID->state_a[0] = *current_error;
	  PID->state_a[2] = diff;
	  PID->state_a[3] = diff_filt;
	  PID->int_term = int_term;
}
