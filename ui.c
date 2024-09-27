#include <stdio.h>
#include <stdint.h>
#include <math.h>
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
char droneChar(int x, int y) {
	if (x >= 0 && x < 64 && y >= 0 && y < DRONE_IMAGE_HEIGHT) {
		return (droneImage[y] & 0x8000000000000000 >> x) ? '#' : ' ';
	} else {
		return ' ';
	}
}

int setGraphBufferChar(int x, int y, char value) {
	graphBuffer[(GRAPH_WIDTH + 1) * y + x] = value;
	return 0;
}

char getGraphBufferChar(int x, int y) {
	return graphBuffer[(GRAPH_WIDTH + 1) * y + x];
}

int clearGraphBuffer() {
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

int renderGraphBuffer(double rollAngle) {
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

int setAnimationBufferChar(int x, int y, char value) {
	animationBuffer[(ANIMATION_WIDTH + 1) * y + x] = value;
	return 0;
}

int clearAnimationBuffer() {
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

int renderAnimationBuffer(double rollAngle) {
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

int main(void) {
	double time = 0.0;
	clearAnimationBuffer();
	clearGraphBuffer();
	clearScreen();
	int fb = open("/dev/fb0", O_RDWR);
	uint32_t ioctlDummy = 0;
	while (1) {
		time += 1.0/30.0;
		double rollAngle = sin(time)/2.0;
		renderAnimationBuffer(rollAngle);
		renderGraphBuffer(rollAngle);
		int error_code = ioctl(fb, FBIO_WAITFORVSYNC, &ioctlDummy);
		resetCursor();
		puts(animationBuffer);
		puts("");
		puts(graphBuffer);
	}
	clearScreen();
	return 0;
}
