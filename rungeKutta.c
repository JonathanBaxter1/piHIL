#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <pthread.h>
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#define clearScreen() printf("\x1b[H\x1b[2J\x1b[3J")
#define resetCursor() printf("\x1b[H\x1b[3J")
#define ANIMATION_WIDTH 80
#define ANIMATION_HEIGHT 32
#define ANIMATION_BUFFER_LEN ((ANIMATION_WIDTH+1) * ANIMATION_HEIGHT) // +1 for newline

#define GRAPH_WIDTH 80
#define GRAPH_HEIGHT 32
#define GRAPH_BUFFER_LEN ((GRAPH_WIDTH+1) * GRAPH_HEIGHT) // +1 for newline

#define DRONE_IMAGE_HEIGHT 32

#define ODE_ORDER 4
#define STOP_TIME 20.0
#define STEP_SIZE 1.0/1024.0

#define HOVER_THROTTLE 0.5

#define PI 3.14159265358979323846
#define G 9.81

char animationBuffer[ANIMATION_BUFFER_LEN];
char graphBuffer[GRAPH_BUFFER_LEN];
uint64_t droneImage[DRONE_IMAGE_HEIGHT] = {
	0,0,0,0,0,0,0,0,0,0,0,
	0x03000000000000C0,
	0xFFFC000000003FFF,
	0x03000000000000C0,
	0x078000FFFF0001E0,
	0x07FFFFFFFFFFFFE0,
	0x030FFFFFFFFFF0C0,
	0x00000DFFFFB00000,
	0x000018FFFF180000,
	0x0000307FFE0C0000,
	0x0000600000060000,
	0,0,0,0,0,0,0,0,0,0,0
};

// Gets one char from the drone image
char droneChar(int x, int y)
{
	if (x >= 0 && x < 64 && y >= 0 && y < DRONE_IMAGE_HEIGHT) {
		return (droneImage[y] & 0x8000000000000000 >> x) ? '#' : ' ';
	} else {
		return ' ';
	}
}

int setGraphBufferChar(int x, int y, char value)
{
	graphBuffer[(GRAPH_WIDTH + 1) * y + x] = value;
	return 0;
}

char getGraphBufferChar(int x, int y)
{
	return graphBuffer[(GRAPH_WIDTH + 1) * y + x];
}

int clearGraphBuffer()
{
	for (int bufferPos = 0;  bufferPos < GRAPH_BUFFER_LEN; bufferPos++) {
		graphBuffer[bufferPos] = ' ';
	}
	for (int x = 1; x < GRAPH_WIDTH-1; x++) {
		int bufferOffset = GRAPH_BUFFER_LEN/2-GRAPH_WIDTH;
		graphBuffer[x + bufferOffset] = '-';
	}
	for (int y = 0; y < GRAPH_HEIGHT; y++) {
		graphBuffer[y*(GRAPH_WIDTH+1)+GRAPH_WIDTH-1] = '|';
		graphBuffer[y*(GRAPH_WIDTH+1)+GRAPH_WIDTH] = '\n';
	}

//	graphBuffer[GRAPH_BUFFER_LEN/2-GRAPH_WIDTH] = '<';
	graphBuffer[GRAPH_BUFFER_LEN/2-2] = '+';
	graphBuffer[GRAPH_BUFFER_LEN-1] = '\0';
	return 0;
}

int renderGraphBuffer(double rollAngle)
{
	for (int x = 1; x < GRAPH_WIDTH-1; x++) {
		for (int y = 0; y < GRAPH_HEIGHT-1; y++) {
			setGraphBufferChar(x-1, y, getGraphBufferChar(x, y));
		}
	}
	for (int y = 0; y < GRAPH_HEIGHT; y++) {
		setGraphBufferChar(GRAPH_WIDTH-2, y, ' ');
	}
	int yPlotPos = rollAngle*GRAPH_HEIGHT/2.0 + GRAPH_HEIGHT/2;
	setGraphBufferChar(GRAPH_WIDTH-2, GRAPH_HEIGHT/2-1, '-');
	setGraphBufferChar(GRAPH_WIDTH-2, yPlotPos, '#');
	return 0;
}

int setAnimationBufferChar(int x, int y, char value)
{
	animationBuffer[(ANIMATION_WIDTH + 1) * y + x] = value;
	return 0;
}

int clearAnimationBuffer()
{
	for (int bufferPos = 0;  bufferPos < ANIMATION_BUFFER_LEN; bufferPos++) {
		int x = bufferPos % (ANIMATION_WIDTH + 1);
		int y = bufferPos / (ANIMATION_WIDTH + 1);
		if (x == ANIMATION_WIDTH) {
			animationBuffer[bufferPos] = '\n';
		} else if (x == 0 || x == ANIMATION_WIDTH - 1) {
			animationBuffer[bufferPos] = '|';
		} else if (y == 0 || y == ANIMATION_HEIGHT - 1) {
			animationBuffer[bufferPos] = '-';
		} else {
			animationBuffer[bufferPos] = ' ';
		}
	}
	animationBuffer[ANIMATION_BUFFER_LEN-1] = '\0';
	return 0;
}

// Segfault when drone rotates 180 degrees, need to fix (change)
int renderAnimationBuffer(double rollAngle)
{
	for (int y = 1; y < ANIMATION_HEIGHT - 1; y++) {
		for (int x = 1; x < ANIMATION_WIDTH - 1; x++) {
			float xCentered = x - ANIMATION_WIDTH/2;
			float yCentered = (y - ANIMATION_HEIGHT/2)*2;
			double cosAngle = cos(rollAngle); // double sin/cos operations take ~ 70 clock cycles
			double sinAngle = sin(rollAngle);
			int xRotated = round(cosAngle*xCentered - sinAngle*yCentered);
			int yRotated = round(sinAngle*xCentered + cosAngle*yCentered);
			int xFinal = xRotated + 32;
			int yFinal = yRotated/2 + DRONE_IMAGE_HEIGHT/2;
			char curChar = droneChar(xFinal, yFinal);
			setAnimationBufferChar(x, y, curChar);
		}
	}
	return 0;
}

void error(char message[])
{
	printf("\033[31merror: "); // set color to red
	puts(message);
	printf("\033[0m"); // reset color
}

uint64_t getTime(void)
{
	uint64_t curTime;
	asm volatile("mrs %0, cntvct_el0" : "=r"(curTime));
	return curTime;
}

// Should be 54 Mhz on Rpi 4
uint64_t getClockFreq(void)
{
	uint64_t clockFreq;
	asm volatile("mrs %0, cntfrq_el0" : "=r"(clockFreq));
	return clockFreq;
}

void arrayTimesScalar(double *arrayOut, double *arrayIn, double scalar, int size)
{
	for (int i = 0; i < size; i++) {
		arrayOut[i] = arrayIn[i]*scalar;
	}
}

void arrayPlusArray(double *arrayOut, double *arrayIn1, double *arrayIn2, int size)
{
	for (int i = 0; i < size; i++) {
		arrayOut[i] = arrayIn1[i] + arrayIn2[i];
	}
}

// Sets a minimum and maximum bound for a value
double saturation(double value, double min, double max)
{
	if (value >= max) {
		return max;
	} else if (value <= min) {
		return min;
	} else {
		return value;
	}
}

// Simulates the MPU-9250's Accerlerometer
double accelerometer(double rollAngle)
{
	double yAcc = G*sin(rollAngle);
	// Also need to add noise to this (change)
	return yAcc;
}

// Simulates the MPU-9250's Gyroscope
double gyro(double rollRate)
{
	return rollRate; // add noise (change)
}

// Simulates the drone's flight controller
double droneController(double t, double gyroData, double accData)
{
	// PID gains
	double Pgain = 0.05;
	double Igain = 0.0;
	double Dgain = 0.012;

	double reference = (fmodl(t, 6.0) < 3.0)*1.5 - 0.75;
	double rollSignal = asin(accData/G);
	double rollRate = gyroData;
	double pidInput = reference - rollSignal;
	double pidInputD = -rollRate;

	// PID calculation
	double pidOutput = pidInput*Pgain + pidInputD*Dgain;
	return pidOutput;
}

// Drone Dynamics
void odes(double *dy, double t, double y[ODE_ORDER])
{
	double throttle = HOVER_THROTTLE;
	double accData = accelerometer(y[0]);
	double gyroData = gyro(y[1]);
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
	double Cd = 0.53;
//	double Cd = 9.53; // change
	double area = 0.2; // Propeller area
	double rho = 1.225; // Air density (Kg/m^3)

	double damping = y[1]*fabs(y[1])*0.5*rho*Cd*area*droneRadius;

	dy[0] = y[1]; // Derivative of Angular Position
	dy[1] = ((y[2]*2 - y[3]*2)*droneRadius - damping)/momentOfInertia; // Derivative of Angular Velocity
	dy[2] = motorLPFw*(motor1LPFinput-y[2]); // Derivative of Motor 1 thrust
	dy[3] = motorLPFw*(motor2LPFinput-y[3]); // Derivative of Motor 2 thrust
}

// Global Variables & Mutexes
double rollAngleGlobal = 0;
pthread_mutex_t rollAngleLock;

// Threads
void rungeKutta(double y[ODE_ORDER], double stepSize, double stopTime);
void* ui(void* vargp);

// Main thread
int main(void)
{
	pthread_mutex_init(&rollAngleLock, NULL);
	pthread_t uiThreadId;
	pthread_create(&uiThreadId, NULL, ui, NULL);
	double initialConditions[ODE_ORDER] = {0, 0, HOVER_THROTTLE, HOVER_THROTTLE};
	rungeKutta(initialConditions, STEP_SIZE, STOP_TIME);
	pthread_mutex_destroy(&rollAngleLock);
	return 0;
}

// Runge-Kutta ODE Solver. This thread runs on core 1.
void rungeKutta(double y[ODE_ORDER], double stepSize, double stopTime)
{
	double dt = stepSize;
	double k1[ODE_ORDER], k2[ODE_ORDER], k3[ODE_ORDER], k4[ODE_ORDER], dydt[ODE_ORDER];
	double k1a[ODE_ORDER], k2a[ODE_ORDER], k3a[ODE_ORDER]; // Temporary Variables

	// This function uses variable size arrays, which may be a lot slower than doing each
	// individually. Idk though, I haven't looked at the assembly.
	double t = 0; // Time
	struct timespec sleepTime;
	sleepTime.tv_sec = 0;
	sleepTime.tv_nsec = 0;
	uint64_t counterFreq = getClockFreq();
	uint64_t startTime = getTime();
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
//		printf("Time: %.6f, Value: %.20f\n", t, y[0]);
		pthread_mutex_lock(&rollAngleLock);
		rollAngleGlobal = y[0];
		pthread_mutex_unlock(&rollAngleLock);
		t += dt;
		uint64_t curTime = getTime();
		int64_t sleepTimeNs = t*1000000000.0 - (curTime - startTime)*1000000000.0/counterFreq;
		if (sleepTimeNs < 0) {
			error("Runge Kutta Solver cannot keepup. Please increase step size, \nreduce system complexity, or increase cpu speed.");
			sleepTimeNs = 0;
		}
		sleepTime.tv_nsec = sleepTimeNs;
		nanosleep(&sleepTime, NULL);

	}
	printf("\n(Final) Time: %.6f, Value: %.20f\n", t, y[0]);
}

// User interface thread running on core 0 with the OS
void* ui(void* vargp)
{
	double time = 0.0;
	clearAnimationBuffer();
	clearGraphBuffer();
	clearScreen();
	int fb = open("/dev/fb0", O_RDWR);
	uint32_t ioctlDummy = 0;
	double rollAngle = 0;
	while (time < STOP_TIME) {
		time += 1.0/60.0;
//		double rollAngle = sin(time*2.0)/2.0;
		pthread_mutex_lock(&rollAngleLock);
		rollAngle = rollAngleGlobal;
		pthread_mutex_unlock(&rollAngleLock);

		renderAnimationBuffer(rollAngle);
		renderGraphBuffer(rollAngle);
		int error_code = ioctl(fb, FBIO_WAITFORVSYNC, &ioctlDummy);
		resetCursor();
		puts(animationBuffer);
		puts("");
		puts(graphBuffer);
	}
	clearScreen();
	pthread_exit(NULL);
}
