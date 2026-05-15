/*
 * Configuration of Motor Speed Profile at initialization
 */

/*#define MAX_SPEED 10000 
#define MIN_SPEED 3000 
#define MAX_ACCEL 65535
#define MAX_DECEL 65535*/

#define MAX_SPEED 2000  //1000
#define MIN_SPEED 500   //800 
#define MAX_ACCEL 6000
#define MAX_DECEL 6000

/*#define MAX_SPEED 2000 
#define MIN_SPEED 800 
#define MAX_ACCEL 6000
#define MAX_DECEL 6000*/


#define MAX_TORQUE_CONFIG 800//400 					// 400 Selected Value for normal control operation
#define MAX_TORQUE_SWING_UP 800					// 800 Selected Value for Swing Up operation

#define OVERCURRENT_THRESHOLD 2000				// 2000 Selected Value for Integrated Rotary Inverted Pendulum System
#define SHUTDOWN_TORQUE_CURRENT 0				// 0 Selected Value for Integrated Rotary Inverted Pendulum System
#define TORQ_CURRENT_DEFAULT MAX_TORQUE_CONFIG				// Default torque current	
#define MOTOR_STEPS_PER_DEGREE 		8.888889

float get_stepper_angle();
void move_stepper_to(float);
void move_stepper_by(float);
void init_motor();
void motor_task();
