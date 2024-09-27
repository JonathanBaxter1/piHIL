#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <wiringPi.h>

// Get all gpio inputs as a 32 bit value, each pin is one bit
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

// From wiringPi library
static uint8_t gpioToGPLEV [] = {
	13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
	14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
};

// Only work in GPIO mode, and not on Raspberry Pi 5
int fastRead(int pin) {
//	if (*(_wiringPiGpio + gpioToGPLEV [pin]) & (1 << pin)) {
//		return HIGH;
//	} else {
//		return LOW;
//	}
	return (*(_wiringPiGpio + gpioToGPLEV [pin]) & (1 << pin)) != 0;
}

int inPWM(PWMwave *wave, int pin) {
	int pinValue = digitalRead(pin);
//	int pinValue = fastRead(pin);
	if (wave->state == LOW && pinValue == HIGH) {
		wave->state = HIGH;
		wave->curTime = getTime();
		wave->period = wave->curTime - wave->startTime;
		wave->startTime = wave->curTime;
	} else if (wave->state == HIGH && pinValue == LOW) {
		wave->state = LOW;
		wave->curTime = getTime();
		wave->pulseWidth = wave->curTime - wave->startTime;
	}
	return 0;
}

int main(void) {
	struct timespec ts;
	ts.tv_sec = 0;
	ts.tv_nsec = 1000000;
	printf("Starting...\n");
	int inputPin1 = 4;
	int inputPin2 = 17;
	int inputPin3 = 27;
	int inputPin4 = 22;
	PWMwave PWMinfo1 = {0, 0, LOW, 0, 0, 0};
	PWMwave PWMinfo2 = {0, 0, LOW, 0, 0, 0};
	wiringPiSetupGpio();
	pinMode(inputPin1, INPUT);
	pinMode(inputPin2, INPUT);
	pinMode(inputPin3, INPUT);
	pinMode(inputPin4, INPUT);
	pullUpDnControl(inputPin1, PUD_DOWN);
	pullUpDnControl(inputPin2, PUD_DOWN);
	pullUpDnControl(inputPin3, PUD_DOWN);
	pullUpDnControl(inputPin4, PUD_DOWN);
	uint32_t frame = 0;
	uint32_t inputs = 0;
	uint32_t lastInputs = 0;
	uint64_t curTime;
	uint64_t lastTime = getTime();
	struct timespec time1;
	struct timespec time2;
	while (1) {
//		inPWM(&PWMinfo1, inputPin2);
//		inPWM(&PWMinfo2, inputPin3);
		frame++;
		inputs = GET_INPUTS();
//		if (inputs != lastInputs) {
//			int kek = 5; //printf("kek\n");
//		}
		curTime = getTime();
//		if (frame % 100000 == 0) {
//		printf("%d\n", frame);
//		if ((frame & 65535) == 0) {//(frame & 0x0000FFFF) == 0) {
//			printf("PWM info: %d, %d, %d, %d\n", PWMinfo1.pulseWidth, PWMinfo1.period, PWMinfo2.pulseWidth, PWMinfo2.period);
//			printf("printf time: %d\n", time2.tv_nsec - time1.tv_nsec);
//			printf("%d, %d\n", curTime - lastTime, getClockFreq());
//		}
		if (frame > 1000000000) { break; }
		lastTime = curTime;
		lastInputs = inputs;
//		nanosleep(&ts, NULL);
	}
	return curTime-lastTime;
}
