/* ============================================================================================================================ *//**
 * @file       OS_Mutex.c
 * @author     Adam Kowalewski
 * @maintainer Krzysztof Pierczyk (krzysztof.pierczyk@gmail.com)
 * @date       Thursday, 19th May 2022 9:53:09 am
 * @modified   Wednesday, 25th May 2022 9:47:39 pm
 * @project    engineering-thesis
 * @brief      CIFX'es mutexes-related OS functions [implementation]
 * 
 * 
 * @copyright Krzysztof Pierczyk © 2022
 */// ============================================================================================================================= */

/* =========================================================== Includes =========================================================== */

#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "cifxDriver.h"
#include "OS_Includes.h"
#include "OS_Common.h"

/* ========================================================== Static Data ========================================================= */

/// File's context
static const char *context = "os_mutex";

/* ======================================================= Public functions ======================================================= */

void* OS_CreateMutex() {
	
	xTraceDebug(context, "Creating mutex...");

	// Allocate memory for the mutex
	pthread_mutex_t* mtx = (pthread_mutex_t*) malloc(sizeof(pthread_mutex_t));
	if(mtx == NULL)	{
		xTraceGlobalSystemError(context, "Could not malloc mutex");
		return NULL;
	}

	xTraceDebug(context, "Initializing mutex...");

	// Initialize the mutex
	const pthread_mutexattr_t* attr = NULL;
	int ec = pthread_mutex_init(mtx, attr);
	if(ec != 0)	{
		xTraceSystemError(context, ec, "Could not init mutex");
		free(mtx);
		return NULL;
	}

	xTraceDebug(context, "Mutex created successfully");
	return mtx;
}


int OS_WaitMutex(void* pvMutex, uint32_t ulTimeoutMs) {

	assert(pvMutex != NULL);
	xTraceDebug(context, "Getting current time...");

	// Set the timeout
	const struct timespec clock_time = get_clock_time();
	const struct timespec timeout = timespec_add_ms(&clock_time, ulTimeoutMs);

	xTraceDebug(context, "Waiting for mutex...");

	// Wait on mutex
	pthread_mutex_t* mtx = (pthread_mutex_t*) pvMutex;
	const int ec = pthread_mutex_timedlock(mtx, &timeout);
	if(ec != 0)	{
		if(ec == ETIMEDOUT)
			xTraceWarn(context, "Timeout during waiting for mutex");
		else {
			xTraceSystemError(context, ec, "Could not wait for mutex");
			assert(0);
		}

		// Timeout
		return 0;
	}

	xTraceDebug(context, "Mutex locked");

	return 1;
}


void OS_ReleaseMutex(void* pvMutex) {

	assert(pvMutex != NULL);
	xTraceDebug(context, "Releasing mutex...");

	pthread_mutex_t* mtx = (pthread_mutex_t*) pvMutex;
	const int ec = pthread_mutex_unlock(mtx);
	if(ec != 0)	{
		xTraceSystemError(context, ec, "Could not lock mutex");
		assert(0); // This should not happen
	}

	xTraceDebug(context, "Mutex released");
}


void OS_DeleteMutex(void* pvMutex)
{
	assert(pvMutex != NULL);
	xTraceDebug(context, "Deleting mutex...");

	pthread_mutex_t* mtx = (pthread_mutex_t*) pvMutex;
	const int ec = pthread_mutex_destroy(mtx);
	free(mtx);
	if(ec != 0)	{
		xTraceSystemError(context, ec, "Could not destroy mutex");
		assert(0);
		return;
	}

	xTraceDebug(context, "Mutex deleted");
}

/* ================================================================================================================================ */
