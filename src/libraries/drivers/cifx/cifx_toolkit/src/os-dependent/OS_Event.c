/* ============================================================================================================================ *//**
 * @file       OS_Event.c
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Wednesday, 25th May 2022 9:47:39 pm
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

	xTraceDebug(context, "Creating event...");

	// Allocate memory for the semaphore
	sem_t* sem = (sem_t*) malloc(sizeof(sem_t));
	if(sem == NULL)	{
		xTraceGlobalSystemError(context, "Could not create event");
		return NULL;
	}

	xTraceDebug(context, "Initializing event...");

	// Initialize the semaphore
	const int pshared = 0;
	const unsigned int value = 0;
	if(sem_init(sem, pshared, value) == -1)	{
		xTraceGlobalSystemError(context, "Could not initialize event");
		assert(0); // This should not happen
		free(sem);
		return NULL;
	}

	xTraceDebug(context, "Event created successfully");
	return sem;
}


void OS_SetEvent(void* pvEvent) {

	assert(pvEvent != NULL);
	xTraceDebug(context, "Setting an event...");

	// Increment the semaphore
	sem_t* sem = (sem_t*) pvEvent;
	if(sem_post(sem) != 0) {
		xTraceGlobalSystemError(context, "Could not set an event");
		assert(0); // This should not happen
	}

	xTraceDebug(context, "Event set successfully");
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
	xTraceDebug(context, "Getting current time...");

	// Prepare timers
	const struct timespec clock_time = get_clock_time();
	const struct timespec timeout = timespec_add_ms(&clock_time, ulTimeoutMs);

	xTraceDebug(context, "Waiting for an event...");
	sem_t* sem = (sem_t*) pvEvent;

	// Wait for the timeout
	while(sem_timedwait(sem, &timeout) == -1) {
		if(errno != EINTR) {
			if(errno != ETIMEDOUT) {
				xTraceGlobalSystemError(context, "Could not wait for an event");
				assert(0);
				return -1;
			}
			xTraceWarn(context, "Event not signalled, timeout occured");
			return -1;
		}
		xTraceWarn(context, "Waiting for an event interrupted, continuing...");
	}
	xTraceDebug(context, "Event occured");

	return 0;
}

/* ================================================================================================================================ */
