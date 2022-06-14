/* ============================================================================================================================ *//**
 * @file       OS_Time.c
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Wednesday, 25th May 2022 9:47:34 pm
 * @project    engineering-thesis
 * @brief      CIFX'es time-related OS functions [implementation]
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

#include <assert.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include "cifxDriver.h"
#include "OS_Includes.h"

/* ================================================================================================================================ */

/// File's context
static const char *context = "os_time";

/* ======================================================= Static functions ======================================================= */

/**
 * @brief Gets Linux-specific current time from the requested clock
 * 
 * @param clock 
 *    target clock
 * @returns 
 *    filled time structure
 */
static struct timespec get_clock_time(int clock) {

	xTraceDebug(context, "Getting timer from #%d clock ...", clock);

	struct timespec ts;
	if(clock_gettime(clock, &ts) == -1)	{
		xTraceGlobalSystemError(context, "Could not get millisecond counter value");
		assert(0);
	}
	
	return ts;
}


/**
 * @brief Converts Linux-specific time representation to [ms]
 * 
 * @param ts 
 *    filled time structure
 * @returns 
 *    time in [ms]
 */
static inline uint32_t timespec_to_ms(const struct timespec* ts) {
	return (ts->tv_sec*1000 + ts->tv_nsec/1000000);
}

/* ======================================================= Public functions ======================================================= */

void OS_Sleep(uint32_t ulSleepTimeMs) {

	xTraceDebug(context, "Sleeping for %u msecs...", ulSleepTimeMs);

	const useconds_t usecs = ulSleepTimeMs * 1000;
	if(usleep(usecs) == -1)
		xTraceGlobalSystemError(context, "Could not sleep");

	xTraceDebug(context, "Sleep success");
}


uint32_t OS_GetMilliSecCounter(void) {

	xTraceDebug(context, "Getting millis counter value...");

	const struct timespec clock_time = get_clock_time(CLOCK_MONOTONIC);
	const uint32_t ms = timespec_to_ms(&clock_time);

	xTraceDebug(context, "Millis counter value is %u", ms);
	return ms;
}


uint32_t OS_Time(uint32_t *ptTime) {

	assert(ptTime != NULL);
	xTraceDebug(context, "Getting system time...");

	time_t t;
	t = time(&t);
	if(t == -1) {
		xTraceGlobalSystemError(context, "Could not get system time");
		assert(0);
		return 0;
	}

	*ptTime = (uint32_t) t;
	return *ptTime;
}

/* ================================================================================================================================ */
