#include <Arduino.h>
#include <stdint.h>

#define MOTOR_STEPPER_CONTROL_MODE_SPEED    0
#define MOTOR_STEPPER_CONTROL_MODE_POSITION 1

class MotorStepper {
public:
	// Constructor
	MotorStepper(uint8_t StepPin, uint8_t DirPin, uint32_t StepPerRevolution);


	// Set mode control SPEED or POSITION
	void set_control_mode(uint8_t Mode);

	// Set order of control, 1 = Constant SPEED, 2 = Constant ACCELERATION, 3 = Constant JERK
	void set_control_order(uint8_t N);

	// Set how many times speed, acceleration, jerk is calculated every second
	void set_control_frequency(float Frequency);

	// Set linear translation distance for each revolution
	void set_meters_per_revolution(float Meter);

	// Invert positive and negative direction (clockwise and anti-clockwise)
	void invert_direction() { direction = !direction; }


	// Set desired position in Steps, could be positive or negative
	void move_absolute_step(int32_t Step);

	// Set desired position in Degrees, could be positive or negative
	void move_absolute_degree(float Degree);

	// Set desired position in Revolutions, could be positive or negative
	void move_absolute_revolution(float Revolution);

	// Set desired position in Meters, could be positive or negative
	void move_absolute_meter(float Meter);


	// Move additional Steps from current position, could be positive or negative
	void move_relative_step(int32_t Step);

	// Move additional Degrees from current position, could be positive or negative
	void move_relative_degree(float Degree);

	// Move additional Revolutions from current position, could be positive or negative
	void move_relative_revolution(float Revolution);

	// Move additional Meters from current position, could be positive or negative
	void move_relative_meter(float Meter);


	// Set speed in RPM for controlling
	void set_desired_angular_speed(float RPM);

	// Set acceleration in RPM/s for controlling
	void set_desired_angular_acceleration(float RPMPS);

	// Set jerk in RPM/s^2 for controlling
	void set_desired_angular_jerk(float RPMPSPS);


	// Set maximum allowed speed in RPM
	void set_max_angular_speed(float RPM);

	// Set maximum allowed acceleration in RPM/s
	void set_max_angular_acceleration(float RPMPS);

	// Set maximum allowed jerk in RPM/s^2
	void set_max_angular_jerk(float RPMPSPS);


	// Set minimum speed when controlling position of 2nd order
	void set_min_angular_speed_for_trapezoid(float RPM);

	// Set when to speed up and down when controlling position of 2nd order (Distance is less than 0.5)
	void set_accelerate_distance_for_trapezoid(float Distance);


	// Set current position to zero
	void reset_position() { startStep = destinationStep = positionStep = 0; }

	// Return current position in Steps
	int32_t get_position_step() const { return positionStep; }

	// Return current position in Degrees
	float get_position_degree() const { return ((float)positionStep * 360.0) / (float)SPR; }

	// Return current position in Revolutions
	float get_position_revolution() const { return (float)positionStep / (float)SPR; }

	// Return current position in Meters
	float get_position_meter() const { return ((float)positionStep * MPR) / (float)SPR; }


	// Return current speed in RPM
	float get_angular_speed() const { return currentDerivatives[0]; }
	
	// Return current acceleration in RPM/s
	float get_angular_acceleration() const { return currentDerivatives[1]; }
	
	// Return current jerk in RPM/s^2
	float get_angular_jerk() const { return currentDerivatives[2]; }


	// Stop the motor
	void stop();

	// Return true if the motor has stopped
	bool is_stopped();

	// This must always be called in loop()
	void control();

private:
	const uint8_t STEP_PIN, DIR_PIN;
	const int32_t SPR;

	bool state = false, direction = false;
	uint8_t order = 1, mode = 0;
	int32_t positionStep = 0, destinationStep = 0, startStep = 0;
	float desiredDerivatives[3] = {0.0, 0.0, 0.0},
				maxDerivatives[3] = {0.0, 0.0, 0.0},
				currentDerivatives[3] = {0.0, 0.0, 0.0},
				speedDesiredPPS = 0.0,
				controlFrequency = 100,
				MPR = 0.0,
				desiredAccelerateDistanceForTrapezoid = 0.2,
				actualAccelerateDistanceForTrapezoid = 0.0,
				minSpeedForTrapezoid = 5.0;
	uint64_t nowMicros = 0, preControl = 0, preMove = 0;
};