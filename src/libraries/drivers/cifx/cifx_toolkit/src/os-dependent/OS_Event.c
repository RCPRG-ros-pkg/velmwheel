/* ============================================================================================================================ *//**
 * @file       OS_Event.c
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Tuesday, 28th June 2022 3:02:01 pm
 * @project    engineering-thesis
 * @brief      CIFX'es events-related OS functions [implementation]
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

#include <assert.h>
#include <errno.h>
#include <semaphore.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include "cifXToolkit.h"
#include "OS_Common.h"
#include "OS_Includes.h"

/* ========================================================== Static Data ========================================================= */

/// File's context
static const char *context = "os_event";

/* ======================================================= Public functions ======================================================= */

void* OS_CreateEvent(void) {

	// Allocate memory for the semaphore
	sem_t* sem = (sem_t*) malloc(sizeof(sem_t));
	if(sem == NULL)	{
		xTraceGlobalSystemError(context, "Could not create event");
		return NULL;
	}

	// Initialize the semaphore
	const int pshared = 0;
	const unsigned int value = 0;
	if(sem_init(sem, pshared, value) == -1)	{
		xTraceGlobalSystemError(context, "Could not initialize event");
		assert(0); // This should not happen
		free(sem);
		return NULL;
	}

	return sem;
}


void OS_SetEvent(void* pvEvent) {

	assert(pvEvent != NULL);

	// Increment the semaphore
	sem_t* sem = (sem_t*) pvEvent;
	if(sem_post(sem) != 0) {
		xTraceGlobalSystemError(context, "Could not set an event");
		assert(0); // This should not happen
	}
}


void OS_ResetEvent(void* pvEvent) {
	(void)(pvEvent);
}


void OS_DeleteEvent(void* pvEvent) {
	sem_t* sem = (sem_t*) pvEvent;
	const int status = sem_destroy(sem);
	assert(status == 0);
	free(pvEvent);
	xTraceDebug(context, "Event deleted");
}


uint32_t OS_WaitEvent(void* pvEvent, uint32_t ulTimeoutMs) {

	assert(pvEvent != NULL);

	// Prepare timers
	const struct timespec clock_time = get_clock_time();
	const struct timespec timeout = timespec_add_ms(&clock_time, ulTimeoutMs);

	sem_t* sem = (sem_t*) pvEvent;

	// Wait for the timeout
	while(sem_timedwait(sem, &timeout) == -1) {
		if(errno != EINTR) {
			if(errno != ETIMEDOUT) {
				xTraceGlobalSystemError(context, "Could not wait for an event");
				assert(0);
				return -1;
			}
			return -1;
		}
		xTraceWarn(context, "Waiting for an event interrupted, continuing...");
	}

	return 0;
}

/* ================================================================================================================================ */
