#include <stdio.h>
#include <math.h>

#define ODE_ORDER 4
#define STOP_TIME 3.5
#define STEP_SIZE 1.0/1024.0

#define HOVER_THROTTLE 0.5

uint64_t getTime(void) {
	uint64_t curTime;
	asm volatile("mrs %0, cntvct_el0" : "=r"(curTime));
	return curTime;
}

// Should be 54 Mhz on Rpi 4
uint64_t getClockFreq(void) {
	uint64_t clockFreq;
	asm volatile("mrs %0, cntfrq_el0" : "=r"(clockFreq));
	return clockFreq;
}

void arrayTimesScalar(double *arrayOut, double *arrayIn, double scalar, int size) {
	for (int i = 0; i < size; i++) {
		arrayOut[i] = arrayIn[i]*scalar;
	}
}

void arrayPlusArray(double *arrayOut, double *arrayIn1, double *arrayIn2, int size) {
	for (int i = 0; i < size; i++) {
		arrayOut[i] = arrayIn1[i] + arrayIn2[i];
	}
}

// Sets a minimum and maximum bound for a value
double saturation(double value, double min, double max) {
	if (value >= max) {
		return max;
	} else if (value <= min) {
		return min;
	} else {
		return value;
	}
}

// Simulates the MPU-9250's Accerlerometer
double accelerometer(double rollAngle) {
	double g = 9.81;
	double yAcc = g*cos(rollAngle);
	// Also need to add noise to this (change)
	return yAcc;
}

// Simulates the MPU-9250's Gyroscope
double gyro(double rollRate) {
	return rollRate; // add noise (change)
}

// Simulates the drone's flight controller
double droneController(double t, double gyroData, double accData) {
	// PID gains
	double Pgain = 0.015;
	double Igain = 0.0;
	double Dgain = 0.0;

	double reference = (fmodl(t, 10.0) < 5.0)*0.75;
	double rollSignal = gyroData;
	double pidInput = reference - rollSignal;

	// PID calculation
	double pidOutput = pidInput*Pgain;
	return pidOutput;
}

// Drone Dynamics
void odes(double *dy, double t, double y[ODE_ORDER]) {
	double throttle = HOVER_THROTTLE;
	double accData = accelerometer(y[0]);
	double gyroData = gyro(y[0]);
	double roll = droneController(t, gyroData, accData);
	double motor1Throttle = throttle + roll; // just get these from drone (change)
	double motor2Throttle = throttle - roll;
	double motor3Throttle = throttle + roll;
	double motor4Throttle = throttle - roll;

	// Check if motors have correct throttle relative to each other, then just use motor1
	// (change)
	double motor1LPFinput = saturation(motor1Throttle - 0.08, 0, 0.9)*22.5;
	double motor2LPFinput = saturation(motor2Throttle - 0.08, 0, 0.9)*22.5;

	double motorLPFw = 7.8; // cutoff angular frequency of LPF that describes motor/prop
	double droneRadius = 0.138; // Meters
	double momentOfInertia = 0.01147; // Kg*m^2
//	double Cd = 0.53;
	double Cd = 9.53; // change
	double area = 0.2; // Propeller area
	double rho = 1.225; // Air density (Kg/m^3)

	double damping = y[1]*fabs(y[1])*0.5*rho*Cd*area*droneRadius;

	dy[0] = y[1]; // Derivative of Angular Position
	dy[1] = ((y[2]*2 - y[3]*2)*droneRadius - damping)/momentOfInertia; // Derivative of Angular Velocity
	dy[2] = motorLPFw*(motor1LPFinput-y[2]); // Derivative of Motor 1 thrust
	dy[3] = motorLPFw*(motor2LPFinput-y[3]); // Derivative of Motor 2 thrust
}

// Runge-Kutta ODE Solver
void rungeKutta(double y[ODE_ORDER], double stepSize, double stopTime) {
	double dt = stepSize;
	double k1[ODE_ORDER], k2[ODE_ORDER], k3[ODE_ORDER], k4[ODE_ORDER], dydt[ODE_ORDER];
	double k1a[ODE_ORDER], k2a[ODE_ORDER], k3a[ODE_ORDER]; // Temporary Variables

	// This function uses variable size arrays, which may be a lot slower than doing each
	// individually. Idk though, I haven't looked at the assembly.
	double t = 0; // Time
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
		printf("Time: %.6f, Value: %.20f\n", t, y[0]);
		t += dt;

	}
	printf("(Final) Time: %.6f, Value: %.20f\n", t, y[0]);
}

int main(void) {
	double initialConditions[ODE_ORDER] = {0, 0, HOVER_THROTTLE, HOVER_THROTTLE};
	rungeKutta(initialConditions, STEP_SIZE, STOP_TIME);
	return 0;
}
