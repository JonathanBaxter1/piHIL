#include <stdio.h>
#include <math.h>

#define ODE_ORDER 4
#define STOP_TIME 5
#define STEP_SIZE 1.0/512.0

#define HOVER_THROTTLE 0.5

void arrayTimesScalar(float *arrayOut, float *arrayIn, float scalar, int size) {
	for (int i = 0; i < size; i++) {
		arrayOut[i] = arrayIn[i]*scalar;
	}
}

void arrayPlusArray(float *arrayOut, float *arrayIn1, float *arrayIn2, int size) {
	for (int i = 0; i < size; i++) {
		arrayOut[i] = arrayIn1[i] + arrayIn2[i];
	}
}

// Sets a minimum and maximum bound for a value
float saturation(float value, float min, float max) {
	if (value >= max) {
		return max;
	} else if (value <= min) {
		return min;
	} else {
		return value;
	}
}

// Simulates the drone's flight controller
float droneController(float t, float gyroData, float accData) {
	float rollSignal = (t > 1 && t < 1.2)*0.1;
	return rollSignal;
}

// Drone Dynamics
void odes(float *dy, float t, float y[ODE_ORDER]) {
	float throttle = HOVER_THROTTLE;
	float roll = droneController(t, 0.0, 0.0);
	float motor1Throttle = throttle + roll; // just get these from drone
	float motor2Throttle = throttle - roll;
	float motor3Throttle = throttle + roll;
	float motor4Throttle = throttle - roll;

	// Check if motors have correct throttle relative to each other, then just use motor1

	float motor1LPFinput = saturation(motor1Throttle - 0.08, 0, 0.9)*22.5;
	float motor2LPFinput = saturation(motor2Throttle - 0.08, 0, 0.9)*22.5;

	float motorLPFw = 7.8; // cutoff angular frequency of LPF that describes motor/prop
	float droneRadius = 0.138; // Meters
	float momentOfInertia = 0.01147; // Kg*m^2
	float Cd = 0.53;
	float area = 0.2; // Propeller area
	float rho = 1.225; // Air density (Kg/m^3)

	float damping = y[1]*fabs(y[1])*0.5*rho*Cd*area*droneRadius;

	dy[0] = y[1]; // Derivative of Angular Position
	dy[1] = ((y[2]*2 - y[3]*2)*droneRadius - damping)/momentOfInertia; // Derivative of Angular Velocity
	dy[2] = motorLPFw*(motor1LPFinput-y[2]); // Derivative of Motor 1 thrust
	dy[3] = motorLPFw*(motor2LPFinput-y[3]); // Derivative of Motor 2 thrust
}

// Runge-Kutta ODE Solver
void rungeKutta(float y[ODE_ORDER], float stepSize, float stopTime) {
	float dt = stepSize;
	float k1[ODE_ORDER], k2[ODE_ORDER], k3[ODE_ORDER], k4[ODE_ORDER], dydt[ODE_ORDER];
	float k1a[ODE_ORDER], k2a[ODE_ORDER], k3a[ODE_ORDER]; // Temporary Variables

	// This function uses variable size arrays, which may be a lot slower than doing each
	// individually. Idk though, I haven't looked at the assembly.
	float t = 0; // Time
	while (t < stopTime) {
		// Calculate 4 approximations for slope
		odes(k1, t, y);
		arrayTimesScalar(k1a, k1, dt/2, ODE_ORDER);
		arrayPlusArray(k1a, k1a, y, ODE_ORDER);
		odes(k2, t + dt/2, k1a);
		arrayTimesScalar(k2a, k2, dt/2, ODE_ORDER);
		arrayPlusArray(k2a, k2a, y, ODE_ORDER);
		odes(k3, t + dt/2, k2a);
		arrayTimesScalar(k3a, k3, dt, ODE_ORDER);
		arrayPlusArray(k3a, k3a, y, ODE_ORDER);
		odes(k4, t + dt, k3a);

		// Use weighted average of these 4 approximations
		for (int i = 0; i < ODE_ORDER; i++) {
			dydt[i] = dt/6*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
			y[i] += dydt[i];
		}
		t += dt;

	}
	printf("Time: %.6f, Value: %.20f\n", t, y[0]);
}

int main(void) {
	float initialConditions[ODE_ORDER] = {0, 0, HOVER_THROTTLE, HOVER_THROTTLE};
	rungeKutta(initialConditions, STEP_SIZE, STOP_TIME);
	return 0;
}
