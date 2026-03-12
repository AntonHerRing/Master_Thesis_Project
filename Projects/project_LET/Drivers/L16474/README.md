
# Task description
This task reads the control signal, actuates the motor to move the corresponding degrees and finally returns the current rotor position.

# Task dependencies
- `steppermotor.h`: Predefined configurations for STSW-EDUKIT01 motor
- `l6474.h`: Implementation of the L6474 Driver
- `l6474_target_config.h`: Predefined values for L6474 registers.
- `motor_rpi3b_interface.h`: Header for BSP driver for Raspberry Pi 3 Model B+ based on L6474
- `motor.h`: Function prototypes for the motor driver (not limited to L6474)
- `registers.h`: Registers for pendulum state and target angle of the rotor
- `logging.h`: Module for logging this task's events

# Tests
- The basic functionalities of the motor driver have been tested by comparing the actual value and the command value.
	- Move the motor of the specified degrees: commanded the motor to move 360 degrees, the rotor in actual rotated one round.
	- Read the absolute position of the rotor in number of degrees: when the motor rotated one round, the rotor position read was 360 degrees sharp.