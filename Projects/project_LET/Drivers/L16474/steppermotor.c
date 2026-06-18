//#pragma GCC optimize ("O0") /* Incldue for dubuggning. Easier viewing of variables */

#include <stdio.h>
#include "l6474.h"
#include "steppermotor.h"
//#include "registers.h"
//#include "logging.h"

#include "bsp.h"

#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/structs/io_bank0.h"


L6474_Init_t gL6474InitParams = {
		MAX_ACCEL,           	/// Acceleration rate in step/s2. Range: (0..+inf).
		MAX_DECEL,           	/// Deceleration rate in step/s2. Range: (0..+inf).
		MAX_SPEED,              /// Maximum speed in step/s. Range: (30..10000].
		MIN_SPEED,              /// Minimum speed in step/s. Range: [30..10000).
		MAX_TORQUE_CONFIG, 		/// Torque regulation current in mA. (TVAL register) Range: 31.25mA to 4000mA.
		OVERCURRENT_THRESHOLD, 	/// Overcurrent threshold (OCD_TH register). Range: 375mA to 6000mA.
		L6474_CONFIG_OC_SD_ENABLE, /// Overcurrent shutwdown (OC_SD field of CONFIG register).
		L6474_CONFIG_EN_TQREG_TVAL_USED, /// Torque regulation method (EN_TQREG field of CONFIG register).
		L6474_STEP_SEL_1_16, 	/// Step selection (STEP_SEL field of STEP_MODE register).
		L6474_SYNC_SEL_1_2, 	/// Sync selection (SYNC_SEL field of STEP_MODE register).
		L6474_FAST_STEP_12us, 	/// Fall time value (T_FAST field of T_FAST register). Range: 2us to 32us.
		L6474_TOFF_FAST_8us, 	/// Maximum fast decay time (T_OFF field of T_FAST register). Range: 2us to 32us.
		3,   					/// Minimum ON time in us (TON_MIN register). Range: 0.5us to 64us.
		21, 					/// Minimum OFF time in us (TOFF_MIN register). Range: 0.5us to 64us.
		L6474_CONFIG_TOFF_044us, /// Target Swicthing Period (field TOFF of CONFIG register).
		L6474_CONFIG_SR_320V_us, /// Slew rate (POW_SR field of CONFIG register).
		L6474_CONFIG_INT_16MHZ, /// Clock setting (OSC_CLK_SEL field of CONFIG register).
		(L6474_ALARM_EN_OVERCURRENT | L6474_ALARM_EN_THERMAL_SHUTDOWN
				| L6474_ALARM_EN_THERMAL_WARNING | L6474_ALARM_EN_UNDERVOLTAGE
				| L6474_ALARM_EN_SW_TURN_ON | L6474_ALARM_EN_WRONG_NPERF_CMD) /// Alarm (ALARM_EN register).
};

// Get the position of the stepper motor (in degrees)
float get_stepper_angle() {

	int32_t pos;
	float deg = 0.0;

	// Get stepper position (in number of steps)
	pos = L6474_GetPosition(0);

	// Convert to degrees
	deg = (float)(pos / MOTOR_STEPS_PER_DEGREE);

	return deg;
}

// Tell the stepper motor to move to a particular angle in degrees
void move_stepper_to(float deg) {
	static bool wait = false;

	int32_t steps = (int32_t)(deg * MOTOR_STEPS_PER_DEGREE);

	/* Tell stepper motor to move */
	if (wait == false)
		L6474_GoTo(0, steps);
	/* Prevent reactivtation while Active */
	if (L6474_GetDeviceState(0) != INACTIVE){
		wait = true;
	}
	else
		wait = false;
	
	//L6474_GoTo(0, steps);
	//L6474_WaitWhileActive(0);
}

// Tell the stepper motor to move by a particular angle in degrees
void move_stepper_by(float deg) {

	motorDir_t stp_dir = FORWARD;
	int32_t steps = (int32_t)(deg * MOTOR_STEPS_PER_DEGREE);

	// Use direction and absolute step counts
	if (steps < 0) {
		steps = -1 * steps;
		stp_dir = BACKWARD;
	}

	// Tell stepper motor to move
	L6474_Move(0, stp_dir, steps);
	L6474_WaitWhileActive(0);
}

// The motor needs to be initialised somewhere at the beginning of the thread
void init_motor() {
	L6474_SetNbDevices(1);
	L6474_Init(&gL6474InitParams);
}

/*void motor_task() {
	float control_signal, rotor_angle;

	control_signal = read_float_register(&control_signal_register);
	move_stepper_by(control_signal);

	rotor_angle = get_stepper_angle();

	char buf[80];
    sprintf(buf, "Current rotor angle: %.2f", rotor_angle);
    //system_log("Motor task:", buf);
	
	write_float_register(rotor_angle, &rotor_angle_register);
	//write_float_register(0.0, &control_signal_register);
}*/
