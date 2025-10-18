#include "MotorStepper.h"

MotorStepper::MotorStepper(uint8_t StepPin, uint8_t DirPin, uint32_t StepPerRevolution) : STEP_PIN(StepPin), DIR_PIN(DirPin), SPR(StepPerRevolution) {
	pinMode(STEP_PIN, OUTPUT);
	pinMode(DIR_PIN, OUTPUT);
}


// Set mode control SPEED or POSITION
void MotorStepper::set_control_mode(uint8_t Mode) {
	if (Mode > 1 || mode == Mode) return;
	mode = Mode;
	stop();
}

// Set order of control, 1 = Constant SPEED, 2 = Constant ACCELERATION, 3 = Constant JERK
void MotorStepper::set_control_order(uint8_t N) {
	if (N < 1 || N > 3) return;
	order = N;
}

// Set how many times speed, acceleration, jerk is calculated every second
void MotorStepper::set_control_frequency(float Frequency) {
	if (Frequency <= 0.0) return;
	controlFrequency = Frequency;
}

// Set linear translation distance for each revolution
void MotorStepper::set_meters_per_revolution(float Meter) {
	if (Meter < 0.0) return;
	MPR = Meter;
}


// Set desired position in Steps, could be positive or negative
void MotorStepper::move_absolute_step(int32_t Step) {
	if (!is_stopped()) return;
	destinationStep = Step;
	startStep = positionStep;
}

// Set desired position in Degrees, could be positive or negative
void MotorStepper::move_absolute_degree(float Degree) {
	if (!is_stopped()) return;
	destinationStep = (int32_t)((Degree * (float)SPR) / 360.0);
	startStep = positionStep;
}

// Set desired position in Revolutions, could be positive or negative
void MotorStepper::move_absolute_revolution(float Revolution) {
	if (!is_stopped()) return;
	destinationStep = (int32_t)(Revolution * (float)SPR);
	startStep = positionStep;
}

// Set desired position in Meters, could be positive or negative
void MotorStepper::move_absolute_meter(float Meter) {
	if (!is_stopped()) return;
	destinationStep = (int32_t)((Meter * (float)SPR) / MPR);
	startStep = positionStep;
}


// Move additional Steps from current position, could be positive or negative
void MotorStepper::move_relative_step(int32_t Step) {
	if (!is_stopped()) return;
	destinationStep += Step;
	startStep = positionStep;
}

// Move additional Degrees from current position, could be positive or negative
void MotorStepper::move_relative_degree(float Degree) {
	if (!is_stopped()) return;
	destinationStep += (int32_t)((Degree * (float)SPR) / 360.0);
	startStep = positionStep;
}

// Move additional Revolutions from current position, could be positive or negative
void MotorStepper::move_relative_revolution(float Revolution) {
	if (!is_stopped()) return;
	destinationStep += (int32_t)(Revolution * (float)SPR);
	startStep = positionStep;
}

// Move additional Meters from current position, could be positive or negative
void MotorStepper::move_relative_meter(float Meter) {
	if (!is_stopped()) return;
	destinationStep += (int32_t)((Meter * (float)SPR) / MPR);
	startStep = positionStep;
}


// Set speed in RPM for controlling
void MotorStepper::set_desired_angular_speed(float RPM) {
	if (mode == MOTOR_STEPPER_CONTROL_MODE_SPEED) {
		desiredDerivatives[0] = min(RPM, maxDerivatives[0]);
		desiredDerivatives[0] = max(RPM, -maxDerivatives[0]);
	}
	else if (mode == MOTOR_STEPPER_CONTROL_MODE_POSITION) {
		if (RPM < 0.0) return;
		desiredDerivatives[0] = min(RPM, maxDerivatives[0]);
	}
}

// Set acceleration in RPM/s for controlling
void MotorStepper::set_desired_angular_acceleration(float RPMPS) {
	if (RPMPS < 0.0) return;
	desiredDerivatives[1] = min(RPMPS, maxDerivatives[1]);
}

// Set jerk in RPM/s^2 for controlling
void MotorStepper::set_desired_angular_jerk(float RPMPSPS) {
	if (RPMPSPS < 0.0) return;
	desiredDerivatives[2] = min(RPMPSPS, maxDerivatives[2]);
}


// Set maximum allowed speed in RPM
void MotorStepper::set_max_angular_speed(float RPM) {
	if (RPM < 0.0) return;
	maxDerivatives[0] = RPM;
}

// Set maximum allowed acceleration in RPM/s
void MotorStepper::set_max_angular_acceleration(float RPMPS) {
	if (RPMPS < 0.0) return;
	maxDerivatives[1] = RPMPS;
}

// Set maximum allowed jerk in RPM/s^2
void MotorStepper::set_max_angular_jerk(float RPMPSPS) {
	if (RPMPSPS < 0.0) return;
	maxDerivatives[2] = RPMPSPS;
}


// Set minimum speed when controlling position of 2nd order
void MotorStepper::set_min_angular_speed_for_trapezoid(float RPM) {
	if (RPM < 0.0) return;
	minSpeedForTrapezoid = RPM;
}

// Set when to speed up and down when controlling position of 2nd order (Distance is between 0.0 and 0.5)
void MotorStepper::set_accelerate_distance_for_trapezoid(float Distance) {
	if (Distance < 0.0 || Distance > 0.5) return;
	desiredAccelerateDistanceForTrapezoid = Distance;
}


// Stop the motor
void MotorStepper::stop() {
	currentDerivatives[0] = currentDerivatives[1] = currentDerivatives[2] = 0.0;

	if (mode == MOTOR_STEPPER_CONTROL_MODE_SPEED) {
		desiredDerivatives[0] = speedDesiredPPS = 0.0;
		actualAccelerateDistanceForTrapezoid = 0.0;
	}
	else if (mode == MOTOR_STEPPER_CONTROL_MODE_POSITION) {
		speedDesiredPPS = 0.0;
		destinationStep = positionStep;
	}
}

// Return true if the motor has stopped
bool MotorStepper::is_stopped() {
	if (mode == MOTOR_STEPPER_CONTROL_MODE_POSITION) {
		return destinationStep == positionStep;
	}
	else if (mode == MOTOR_STEPPER_CONTROL_MODE_SPEED) {
		return currentDerivatives[0] == 0.0 && currentDerivatives[1] == 0.0 && currentDerivatives[2] == 0.0 && speedDesiredPPS == 0.0;
	}
}

// This must always be called in loop()
void MotorStepper::control() {
	nowMicros = (uint64_t)micros();

	// When to update speed
	if (nowMicros - preControl >= 1e6 / controlFrequency) {
		preControl = nowMicros;

		if (mode == MOTOR_STEPPER_CONTROL_MODE_SPEED && currentDerivatives[0] != desiredDerivatives[0]) {
			if (order == 1) {
				currentDerivatives[0] = desiredDerivatives[0];
			}
			else {
				if (desiredDerivatives[0] > currentDerivatives[0])
					currentDerivatives[order-1] = desiredDerivatives[order-1];
				else if (desiredDerivatives[0] < currentDerivatives[0])
					currentDerivatives[order-1] = -desiredDerivatives[order-1];

				// Calculate acceleration when using 3rd order
				if (order == 3) {
					currentDerivatives[1] += currentDerivatives[2] / controlFrequency;
					if (currentDerivatives[2] > 0)
						currentDerivatives[1] = min(currentDerivatives[1], desiredDerivatives[1]);
					else
						currentDerivatives[1] = max(currentDerivatives[1], -desiredDerivatives[1]);
				}

				// Calculate speed
				currentDerivatives[0] += currentDerivatives[1] / controlFrequency;
				if (currentDerivatives[order-1] > 0)
					currentDerivatives[0] = min(currentDerivatives[0], desiredDerivatives[0]);
				else
					currentDerivatives[0] = max(currentDerivatives[0], desiredDerivatives[0]);
			}

			if (currentDerivatives[0] == desiredDerivatives[0]) {
				currentDerivatives[1] = 0.0;
				currentDerivatives[2] = 0.0;
			}
		}
		else if (mode == MOTOR_STEPPER_CONTROL_MODE_POSITION && positionStep != destinationStep) {
			if (order == 1) { // Rectangle motion profile
				if (destinationStep > positionStep)
					currentDerivatives[0] = desiredDerivatives[0];
				else
					currentDerivatives[0] = -desiredDerivatives[0];
			}
			else if (order == 2) { // Trapezoid motion profile
				int32_t travelledJourney = abs(positionStep - startStep),
								wholeJourney = abs(destinationStep - startStep);

				if (actualAccelerateDistanceForTrapezoid == 0.0) {
					if (travelledJourney < desiredAccelerateDistanceForTrapezoid*wholeJourney) {
						if (destinationStep > positionStep)
							currentDerivatives[1] = desiredDerivatives[1];
						else
							currentDerivatives[1] = -desiredDerivatives[1];
					}
					else if (travelledJourney >= (1.0-desiredAccelerateDistanceForTrapezoid)*wholeJourney) {
						if (destinationStep > positionStep)
							currentDerivatives[1] = -desiredDerivatives[1];
						else
							currentDerivatives[1] = desiredDerivatives[1];
					}
					else
						currentDerivatives[1] = 0;


					if (currentDerivatives[0] == desiredDerivatives[0])
						actualAccelerateDistanceForTrapezoid = travelledJourney / wholeJourney;
				}
				else {
					if (travelledJourney >= (1.0-actualAccelerateDistanceForTrapezoid)*wholeJourney) {
						if (destinationStep > positionStep)
							currentDerivatives[1] = -desiredDerivatives[1];
						else
							currentDerivatives[1] = desiredDerivatives[1];
					}
					else
						currentDerivatives[1] = 0;
				}

				// Calculate speed
				currentDerivatives[0] += currentDerivatives[1] / controlFrequency;

				if (destinationStep > positionStep) {
					currentDerivatives[0] = min(currentDerivatives[0], desiredDerivatives[0]);
					currentDerivatives[0] = max(currentDerivatives[0], minSpeedForTrapezoid);
				}
				else {
					currentDerivatives[0] = max(currentDerivatives[0], -desiredDerivatives[0]);
					currentDerivatives[0] = min(currentDerivatives[0], -minSpeedForTrapezoid);
				}
			}
		}

		speedDesiredPPS = (currentDerivatives[0] * (float)SPR) / 60.0;

		if (direction)
			digitalWrite(DIR_PIN, speedDesiredPPS > 0.0);
		else
			digitalWrite(DIR_PIN, speedDesiredPPS < 0.0);
	}

	// When to move a step
	if (speedDesiredPPS != 0.0 && nowMicros - preMove >= 1e6 / (abs(speedDesiredPPS)*2)) {
		preMove = nowMicros;

		digitalWrite(STEP_PIN, state);
		state = !state;

		if (!state)
			positionStep += (speedDesiredPPS > 0) ? 1 : -1;

		if (mode == MOTOR_STEPPER_CONTROL_MODE_POSITION && positionStep == destinationStep) {
			stop();
		}
	}
}