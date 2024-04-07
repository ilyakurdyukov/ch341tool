#define _GNU_SOURCE 1

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>

extern "C" {

void Arduino_yield(void) { usleep(1000); }
void Arduino_delay(unsigned long ms) { usleep(1000 * ms); }

#include <time.h>
#include <sys/time.h>
unsigned long Arduino_millis(void) {
	static char init = 0;
	static unsigned long first;
	unsigned long ret;
	struct timeval time;
	gettimeofday(&time, NULL);
	ret = time.tv_sec * (int64_t)1000 + time.tv_usec / 1000;
	if (!init) init = 1, first = ret;
	return ret - first;
}

}
