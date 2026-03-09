
# Task description
This task reads the pendulum position in number of steps and angles from the encoder driver, then updates to the registers.

# Task dependencies
- `RED.h`: Implementation of the rotary encoder driver
- `registers.h`: Registers for pendulum state and target angle of the rotor
- `logging.h`: Module for logging this task's events

# Tests
- The basic functionalities of the encoder driver have been tested manually
	- Get the current pendulum position: rotated the pendulum back and forth, when it was back to the starting point, the output value was 0.