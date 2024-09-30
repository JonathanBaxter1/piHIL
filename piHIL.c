#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <wiringPi.h>
#include <pthread.h>

// Gpio inputs/outputs are a 32 bit value; each pin is one bit
#define OUTPUT_SET (*(_wiringPiGpio + 7))
#define OUTPUT_CLR (*(_wiringPiGpio + 10))
#define GET_INPUTS() (*(_wiringPiGpio + 13))
#define GET_INPUTS2() (*(_wiringPiTimer))

typedef struct {
	uint64_t pulseWidth; // Maximum pulse width of 585 years
	uint64_t period;
	int state;
	int update;
//	struct timespec startTime;
//	struct timespec curTime;
	uint64_t startTime;
	uint64_t curTime;
}
PWMwave;

uint64_t getTime(void) {
	uint64_t curTime;
	asm volatile("mrs %0, cntvct_el0" : "=r"(curTime));
	return curTime;
}

uint64_t getClockFreq(void) {
	uint64_t clockFreq;
	asm volatile("mrs %0, cntfrq_el0" : "=r"(clockFreq));
	return clockFreq;
}

// Only work in GPIO mode, and not on Raspberry Pi 5
int fastRead(int pin) {
//	if (*(_wiringPiGpio + gpioToGPLEV [pin]) & (1 << pin)) {
//		return HIGH;
//	} else {
//		return LOW;
//	}
	return (*(_wiringPiGpio + 13) & (1 << pin)) != 0;
}

int inPWM(PWMwave *wave, uint32_t gpioData, uint64_t time, int pin) {
//	int pinValue = digitalRead(pin);
//	int pinValue = fastRead(pin);
	int pinValue = (gpioData & (1 << pin)) != 0;
	if (wave->state == LOW && pinValue == HIGH) {
		wave->state = HIGH;
		wave->curTime = time;
		wave->period = wave->curTime - wave->startTime;
		wave->startTime = wave->curTime;
	} else if (wave->state == HIGH && pinValue == LOW) {
		wave->state = LOW;
		wave->curTime = time;
		wave->pulseWidth = wave->curTime - wave->startTime;
	}
	return 0;
}

// Global Variables and Mutexes
uint32_t gpioGlobal = 0;
uint64_t timeGlobal = 0;
pthread_mutex_t gpioLock;

// Threads
void* ioHandler(void* vargp);

int main(void) {
	wiringPiSetupGpio();
	struct timespec ts;
	ts.tv_sec = 0;
	ts.tv_nsec = 1000000;
	printf("Starting...\n");
	uint32_t frame = 0;
	uint32_t inputs = 0;
	uint32_t lastInputs = 0;
	uint64_t curTime;
	uint64_t lastTime = getTime();
	struct timespec time1;
	struct timespec time2;
	pthread_mutex_init(&gpioLock, NULL);
	pthread_t ioThreadId;
	pthread_create(&ioThreadId, NULL, ioHandler, NULL);
	while (1) {
		inputs = GET_INPUTS();
		if (inputs != lastInputs) {
			pthread_mutex_lock(&gpioLock);
			gpioGlobal = inputs;
			timeGlobal = getTime();
			pthread_mutex_unlock(&gpioLock);
		}
		lastInputs = inputs;
	}
	pthread_mutex_destroy(&gpioLock);
//	while (1) {
//		inPWM(&PWMinfo1, inputPin2);
//		inPWM(&PWMinfo2, inputPin3);
//		frame++;
//		inputs = GET_INPUTS();
//		if (inputs != lastInputs) {
//			int kek = 5; //printf("kek\n");
//		}
//		curTime = getTime();
//		if (frame % 100000 == 0) {
//		printf("%d\n", frame);
//		if ((frame & 65535) == 0) {//(frame & 0x0000FFFF) == 0) {
//			printf("PWM info: %d, %d, %d, %d\n", PWMinfo1.pulseWidth, PWMinfo1.period, PWMinfo2.pulseWidth, PWMinfo2.period);
//			printf("printf time: %d\n", time2.tv_nsec - time1.tv_nsec);
//			printf("%d, %d\n", curTime - lastTime, getClockFreq());
//		}
//		if (frame > 1000000000) { break; }
//		lastTime = curTime;
//		lastInputs = inputs;
//		nanosleep(&ts, NULL);
//	}
	return 0;
}

void* ioHandler(void* vargp)
{
	uint32_t gpio = 0;
	uint32_t lastGpio = 0;
	uint64_t time = 0;
	uint64_t lastTime = 0;
	int inputPin1 = 4;
	int inputPin2 = 17;
	int inputPin3 = 27;
	int outputPin1 = 22;
	pinMode(inputPin1, INPUT);
	pinMode(inputPin2, INPUT);
	pinMode(inputPin3, INPUT);
	pinMode(outputPin1, OUTPUT);
	pullUpDnControl(inputPin1, PUD_DOWN);
	pullUpDnControl(inputPin2, PUD_DOWN);
	pullUpDnControl(inputPin3, PUD_DOWN);
//	pullUpDnControl(inputPin4, PUD_DOWN);
	PWMwave PWMinfo1 = {0, 0, LOW, 0, 0, 0};
	uint64_t clockFreq = getClockFreq();

	while (1) {
		uint64_t curTime = getTime();

		// Mutex
		pthread_mutex_lock(&gpioLock);
		gpio = gpioGlobal;
		time = timeGlobal;
		pthread_mutex_unlock(&gpioLock);

		inPWM(&PWMinfo1, gpio, time, inputPin2);
		if (curTime % clockFreq < clockFreq >> 1) {
			OUTPUT_SET = 1 << outputPin1;
		} else {
			OUTPUT_CLR = 1 << outputPin1;
		}
		if (gpio != lastGpio) {
//			printf("Time difference: %d\n", time - lastTime);
//			printf("Value: %x\n", gpio);
			printf("Period: %d, Pulse width: %d\n", PWMinfo1.period, PWMinfo1.pulseWidth);
		}
		lastGpio = gpio;
		lastTime = time;
	}
	pthread_exit(NULL);
}
